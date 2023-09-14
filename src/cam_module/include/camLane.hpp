#ifndef CAMLANE_HPP_
#define CAMLANE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"

#include <torch/script.h> 
#include <torch/torch.h>

#include <iostream>
#include <vector>
#include <string>
#include <utility> 

using std::placeholders::_1;
class camLane : public rclcpp::Node {
public:
    camLane(const std::string &name);
    ~camLane();

    void shortImagecallback(const sensor_msgs::msg::Image::SharedPtr image);

    void forward(const cv::Mat &_image);

    std::vector<std::vector<std::pair<int, int>>> pred2coords(const c10::impl::GenericDict &pred);

private:
    // rclcpp::callback_group::CallbackGroup::SharedPtr shortImagecallbackGroup;
    // cv_bridge::CvImagePtr cvPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr shortImagesub;

    // 原图片参数
    int ori_img_w = 640;
    int ori_img_h = 480;

    // 模型训练参数
    int train_width= 1600;
    int train_height= 320;
    
    // 裁切比
    float crop_ratio = 0.6;

    // cut_height = (int)(train_height * (1 - crop_ratio))
    int cut_height = 128;

    int num_row= 72;
    int num_col= 81;

    // if cuda available
    bool cudaAvailable;

    
protected:
    torch::jit::script::Module module;
};

#endif // CAMLANE_HPP_