#ifndef CAMLANE_HPP_
#define CAMLANE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"

#include "string.h"

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"

#include <torch/script.h> 
#include <torch/torch.h>

using std::placeholders::_1;
class camLane : public rclcpp::Node {
public:
    camLane(const std::string &name);
    ~camLane();

    void shortImagecallback(const sensor_msgs::msg::Image::SharedPtr image);

    void pred2coords();

private:
    // rclcpp::callback_group::CallbackGroup::SharedPtr shortImagecallbackGroup;
    // cv_bridge::CvImagePtr cvPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr shortImagesub;

protected:
    torch::jit::script::Module module;

};

#endif // CAMLANE_HPP_