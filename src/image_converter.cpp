#include "theta_s_ros/image_converter.hpp"

#include "opencv_theta_s/EquirectangularConversion/ThetaConversion.hpp"
#include "panorama2cubemap/src/Panorama2Cubemap.hpp"

namespace theta_s_ros {
ImageConverter::ImageConverter()
    : private_nh_("~")
{
    using namespace std::literals::string_literals;

    private_nh_.param("crop_y", crop_y_, std::vector<int>());
    private_nh_.param("unmerge_top_and_bottom", unmerge_top_and_bottom_, false);
    if (crop_y_.size() != 0 && crop_y_.size() != 2) {
        ROS_ERROR("crop_y_ must be a list of size 2");
        ros::shutdown();
    }

    image_sub_ = nh_.subscribe(
        "/camera/image_raw", 1, &ImageConverter::image_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    equirectangular_image_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("/equirectangular/image_raw/compressed", 1);
    for (const auto& face : { "right"s, "left"s, "top"s, "bottom"s, "front"s, "back"s, "merged"s }) {
        cubemap_image_pubs_.push_back(nh_.advertise<sensor_msgs::CompressedImage>(
            "/cubemap/" + face + "/image_raw/compressed", 1));
    }
}

void ImageConverter::convert_to_equirectangular(cv::Mat& cv_image, cv::Mat& equirectangular_image)
{
    static ThetaConversion theta_conversion(cv_image.cols, cv_image.rows);
    theta_conversion.doConversion(cv_image);
    static auto rect = crop_y_.size() == 0 ? cv::Rect(0, 0, cv_image.cols, cv_image.rows)
                                           : cv::Rect(0, crop_y_[0], cv_image.cols, crop_y_[1] - crop_y_[0] + 1);
    cv::Mat cv_image_cropped(cv_image, rect);
    equirectangular_image = std::move(cv_image_cropped);
}

void ImageConverter::publish_image(const cv::Mat& src_image, ros::Publisher pub, const std_msgs::Header& header)
{
    sensor_msgs::CompressedImage image_msg;
    image_msg.header = header;
    std::vector<unsigned char> buffer;
    image_msg.format = "jpeg";
    cv::imencode(".jpg", src_image, buffer);
    image_msg.data.swap(buffer);
    pub.publish(image_msg);
}

void ImageConverter::image_callback(const sensor_msgs::ImageConstPtr& received_image)
{
    ROS_DEBUG("start image_callback");
    double start_time = ros::Time::now().toSec();

    ROS_DEBUG("ros image to cv image");
    cv_bridge::CvImageConstPtr cv_image_ptr;
    try {
        cv_image_ptr = cv_bridge::toCvCopy(received_image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& ex) {
        ROS_ERROR("cv_bridge exception: %s", ex.what());
        return;
    }
    cv::Mat cv_image(cv_image_ptr->image.rows, cv_image_ptr->image.cols, cv_image_ptr->image.type());
    cv_image = cv_image_ptr->image;

    ROS_DEBUG("image conversion");
    cv::Mat equirectangular_image;
    std::vector<cv::Mat> cubemap_images;
    cv::Mat merged_image;
    convert_to_equirectangular(cv_image, equirectangular_image);
    pano2cube(cv_image, cubemap_images, merged_image, unmerge_top_and_bottom_);
    cubemap_images.push_back(merged_image);

    ROS_DEBUG("publish images");
    publish_image(equirectangular_image, equirectangular_image_pub_, received_image->header);
    for (std::size_t i = 0; i < cubemap_image_pubs_.size(); ++i) {
        publish_image(cubemap_images[i], cubemap_image_pubs_[i], received_image->header);
    }

    ROS_INFO("[image_converter:image_callback] elapsed time : %f [sec]", ros::Time::now().toSec() - start_time);

    return;
}

void ImageConverter::process()
{
    ROS_DEBUG("start process");
    ros::spin();
    return;
}
} // namespace image_converter

int main(int argc, char** argv)
{
    ros::init(argc, argv, "convert_ros_image_to_cv_image");
    theta_s_ros::ImageConverter image_convertor;
    image_convertor.process();
    return 0;
}
