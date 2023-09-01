#include "shortCam.hpp"

int main(int argc, char **argv) {

    
    rclcpp::init(argc, argv);

    auto node = std::make_shared<shortCam>("shortCam");     

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
