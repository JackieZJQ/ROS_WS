#include "shortCam.hpp"

int main(int argc, char **argv) {

    
    rclcpp::init(argc, argv);

    auto shortCamnode = std::make_shared<shortCam>("shortCam");
    
    rclcpp::spin(shortCamnode);
    
    rclcpp::shutdown();
    return 0;
}
