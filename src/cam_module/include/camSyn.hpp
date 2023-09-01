#ifndef CAMSYN_HPP_
#define CAMSYN_HPP_

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"

class camSyn : public rclcpp::Node {
public:
    camSyn(const std::string &name);

private:
};

#endif // CAMSYN_HPP_
