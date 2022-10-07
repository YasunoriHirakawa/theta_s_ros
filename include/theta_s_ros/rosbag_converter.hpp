#ifndef ROSBAG_CONVERTER_HPP
#define ROSBAG_CONVERTER_HPP

#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include <boost/program_options.hpp>
#include <filesystem>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <thread>

namespace theta_s_ros {

namespace fs = std::filesystem;
using namespace std::literals::string_literals;

class RosbagConverter {
public:
    RosbagConverter(
        fs::path input_bagfile_path, std::string theta_topic_name, std::array<int, 2> crop_y, bool unmerge_top_and_bottom);
    ~RosbagConverter();
    void convert_to_equirectangular(cv::Mat& cv_image, cv::Mat& equirectangular_image);
    void write_cv_image_to_bag(
        const cv::Mat& cv_image, const std::string& topic_name, const std_msgs::Header& header, const ros::Time& time);
    void process_theta_image(const rosbag::MessageInstance& msg);
    void convert(void);

private:
    std::string theta_topic_name_;
    std::string equirectangular_topic_name_;
    std::vector<std::string> cubemap_topic_names_;
    std::array<int, 2> crop_y_;
    bool unmerge_top_and_bottom_;

    unsigned int n_threads_;
    std::vector<std::unique_ptr<std::thread>> threads_;
    std::mutex mutex_;
    std::unique_ptr<rosbag::Bag> bag_in_;
    std::unique_ptr<rosbag::Bag> bag_out_;
};
} // namespace theta_s_ros

#endif // ROSBAG_CONVERTER_HPP
