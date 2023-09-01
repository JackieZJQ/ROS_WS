#ifndef SHORT_CAM_HPP_
#define SHORT_CAM_HPP_

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

class shortCam : public rclcpp::Node {
public:
    shortCam(const std::string &name);
    
    ~shortCam();

    void readImage();

    void publishImage();

private:
    std::shared_ptr<std::thread> runOpencam;
    std::shared_ptr<std::thread> runPubcam;

    cv::VideoCapture cap;

    cv::Mat image;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_msg_publisher;
};

#endif // SHORT_CAM_HPP_