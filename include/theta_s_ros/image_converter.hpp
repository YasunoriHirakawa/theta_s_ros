#ifndef __IMAGE_CONVERTER_HPP
#define __IMAGE_CONVERTER_HPP

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

namespace theta_s_ros {
class ImageConverter {
public:
    ImageConverter();
    void convert_to_equirectangular(cv::Mat& cv_image, cv::Mat& equirectangular_image);
    void publish_image(const cv::Mat& src_image, ros::Publisher pub, const std_msgs::Header& header);
    void image_callback(const sensor_msgs::ImageConstPtr&);
    void process();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber image_sub_;
    ros::Publisher equirectangular_image_pub_;
    std::vector<ros::Publisher> cubemap_image_pubs_;
    std::vector<int> crop_y_;
    bool unmerge_top_and_bottom_;
};
} // namespace theta_s_ros

#endif // __IMAGE_CONVERTER_HPP
