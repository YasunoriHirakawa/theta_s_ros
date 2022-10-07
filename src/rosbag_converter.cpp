#include "theta_s_ros/rosbag_converter.hpp"

#include "opencv_theta_s/EquirectangularConversion/ThetaConversion.hpp"
#include "panorama2cubemap/src/Panorama2Cubemap.hpp"
#include "sensor_msgs/CompressedImage.h"

namespace theta_s_ros {
RosbagConverter::RosbagConverter(
    fs::path input_bagfile_path, std::string theta_topic_name, std::array<int, 2> crop_y, bool unmerge_top_and_bottom)
    : theta_topic_name_(theta_topic_name)
    , equirectangular_topic_name_()
    , cubemap_topic_names_()
    , crop_y_(crop_y)
    , unmerge_top_and_bottom_(unmerge_top_and_bottom)
    , n_threads_(std::thread::hardware_concurrency())
    , threads_()
    , bag_in_(std::make_unique<rosbag::Bag>())
    , bag_out_(std::make_unique<rosbag::Bag>())
{
    if (!fs::exists(input_bagfile_path)) {
        throw std::runtime_error("bagfile does not exist");
    }
    bag_in_->open(input_bagfile_path, rosbag::bagmode::Read);

    fs::path output_bagfile_path = input_bagfile_path.parent_path() / input_bagfile_path.stem().concat("_converted.bag");
    bag_out_->open(output_bagfile_path, rosbag::bagmode::Write);

    equirectangular_topic_name_ = "equirectangular/image_raw/compressed";
    const std::array<std::string, 7> faces { "right"s, "left"s, "top"s, "bottom"s, "front"s, "back"s, "merged"s };
    for (const auto& face : faces) {
        cubemap_topic_names_.push_back("cubemap/" + face + "/image_raw/compressed");
    }

    std::cout << "Start converting: " << input_bagfile_path << " -> " << output_bagfile_path << std::endl;
}

RosbagConverter::~RosbagConverter()
{
    for (auto& thread : threads_) {
        thread->join();
    }
    bag_in_->close();
    bag_out_->close();

    std::cout << "Finished converting" << std::endl;
}

void RosbagConverter::convert_to_equirectangular(cv::Mat& cv_image, cv::Mat& equirectangular_image)
{
    static ThetaConversion theta_conversion(cv_image.cols, cv_image.rows);
    theta_conversion.doConversion(cv_image);
    static auto rect = crop_y_.size() == 0 ? cv::Rect(0, 0, cv_image.cols, cv_image.rows)
                                           : cv::Rect(0, crop_y_[0], cv_image.cols, crop_y_[1] - crop_y_[0] + 1);
    cv::Mat cv_image_cropped(cv_image, rect);
    equirectangular_image = std::move(cv_image_cropped);
}

void RosbagConverter::write_cv_image_to_bag(
    const cv::Mat& cv_image, const std::string& topic_name, const std_msgs::Header& header, const ros::Time& time)
{
    sensor_msgs::CompressedImage compressed_image;
    compressed_image.header = header;
    compressed_image.format = "jpeg";
    cv::imencode(".jpg", cv_image, compressed_image.data);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        bag_out_->write(topic_name, time, compressed_image);
    }
}

void RosbagConverter::process_theta_image(const rosbag::MessageInstance& msg)
{
    sensor_msgs::CompressedImageConstPtr theta_msg;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        theta_msg = msg.instantiate<sensor_msgs::CompressedImage>();
    }
    if (theta_msg == nullptr) {
        return;
    }

    cv::Mat theta_image = cv::imdecode(theta_msg->data, cv::IMREAD_COLOR);
    cv::Mat equirectangular_image;
    std::vector<cv::Mat> cubemap_images;
    cv::Mat merged_image;
    convert_to_equirectangular(theta_image, equirectangular_image);
    pano2cube(theta_image, cubemap_images, merged_image, unmerge_top_and_bottom_);
    cubemap_images.push_back(merged_image);

    write_cv_image_to_bag(equirectangular_image, equirectangular_topic_name_, theta_msg->header, msg.getTime());
    for (size_t i = 0; i < cubemap_images.size(); ++i) {
        write_cv_image_to_bag(cubemap_images[i], cubemap_topic_names_[i], theta_msg->header, msg.getTime());
    }
}

void RosbagConverter::convert(void)
{
    for (const auto& msg : rosbag::View(*bag_in_)) {
        static unsigned int count_threads = 0;

        if (msg.getTopic() == theta_topic_name_) {
            threads_.emplace_back(new std::thread(&RosbagConverter::process_theta_image, this, msg));
            ++count_threads;
        } else if (
            msg.getTopic() != equirectangular_topic_name_
            && std::find(cubemap_topic_names_.begin(), cubemap_topic_names_.end(), msg.getTopic()) == cubemap_topic_names_.end()) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                bag_out_->write(msg.getTopic(), msg.getTime(), msg);
            }
        }

        if (count_threads >= n_threads_) {
            for (auto& thread : threads_) {
                thread->join();
            }
            threads_.clear();
            count_threads = 0;
        }
    }
}
} // namespace theta_s_ros

int main(int argc, char** argv)
{
    namespace po = boost::program_options;
    po::options_description description("Allowed options");
    description.add_options() // clang-format off
        ("help,h", "produce help message")
        ("input", po::value<std::string>(), "input bagfile")
        ("topic", po::value<std::string>()->default_value("theta_s/image_raw/compressed"), "topic name of theta image")
        ("crop", po::value<std::vector<int>>()->multitoken()->default_value({145, 475}, "145 475"), "crop y")
        ("unmerge", po::bool_switch()->default_value(false), "unmerge top and bottom"); // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, description), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << description << std::endl;
        return 1;
    }

    theta_s_ros::RosbagConverter converter(
        vm["input"].as<std::string>(),
        vm["topic"].as<std::string>(),
        { vm["crop"].as<std::vector<int>>()[0], vm["crop"].as<std::vector<int>>()[1] },
        vm["unmerge"].as<bool>());
    converter.convert();

    return 0;
}
