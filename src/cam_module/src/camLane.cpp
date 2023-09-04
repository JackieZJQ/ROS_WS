#include "camLane.hpp"

camLane::camLane(const std::string &name):Node(name) {

    RCLCPP_INFO(this->get_logger(), "---车道线检测模块初始化---");

    // shortImagecallbackGroup = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    shortImagesub = this->create_subscription<sensor_msgs::msg::Image>("short_focal_image", 10, std::bind(&camLane::shortImagecallback, this, _1));

    cv::namedWindow("Test Window", 1);
    
}

camLane::~camLane() {

    cv::destroyAllWindows();
}

void camLane::shortImagecallback(const sensor_msgs::msg::Image::SharedPtr image) {

    try {
        
        auto cvPtr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

        cv::imshow("Test Window", cvPtr->image);
        cv::waitKey(5);

    } catch (cv_bridge::Exception& e) {
      
      // TO-DO ?
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
}