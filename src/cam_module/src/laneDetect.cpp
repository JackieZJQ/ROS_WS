#include "camLane.hpp"

int main(int argc, char **argv) {

    
    rclcpp::init(argc, argv);

    auto camLanenode = std::make_shared<camLane>("camLane");
    
    rclcpp::spin(camLanenode);
    
    rclcpp::shutdown();
    return 0;
}