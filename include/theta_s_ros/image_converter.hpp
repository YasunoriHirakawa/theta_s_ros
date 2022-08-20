#ifndef __IMAGE_CONVERTER_HPP
#define __IMAGE_CONVERTER_HPP

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

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
    ros::Publisher cubemap_image_right_pub_;
    ros::Publisher cubemap_image_left_pub_;
    ros::Publisher cubemap_image_top_pub_;
    ros::Publisher cubemap_image_bottom_pub_;
    ros::Publisher cubemap_image_front_pub_;
    ros::Publisher cubemap_image_back_pub_;
    ros::Publisher cubemap_image_merged_pub_;
    std::vector<int> crop_y_;
};

#endif // __IMAGE_CONVERTER_HPP
