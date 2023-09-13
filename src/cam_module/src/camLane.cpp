#include "camLane.hpp"

camLane::camLane(const std::string &name):Node(name) {

    RCLCPP_INFO(this->get_logger(), "---车道线检测模块初始化---");

    // shortImagecallbackGroup = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    shortImagesub = this->create_subscription<sensor_msgs::msg::Image>("short_focal_image", 10, std::bind(&camLane::shortImagecallback, this, _1));

    cv::namedWindow("Test Window", 1);

    bool cudaAvailable = torch::cuda::is_available();
    if (cudaAvailable) RCLCPP_INFO(this->get_logger(), "---CUDA available! Predicting on GPU---");

    // 加载模型
    // std::string networkName = "";
    // std::cout << "Loading " << networkName << std::endl;
    // module = torch::jit::load(networkName);
    // std::cout << "Loaded" << std::endl;
}

camLane::~camLane() {

    cv::destroyAllWindows();
}

void camLane::shortImagecallback(const sensor_msgs::msg::Image::SharedPtr image) {

    // 设置推理设备
    torch::DeviceType device_type;
    
    if (torch::cuda::is_available() == true) device_type = torch::kCUDA;
    else device_type = torch::kCPU;

    torch::Device device(device_type);

    try {
        
        auto cvPtr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

        cv::imshow("Test Window", cvPtr->image);
        cv::waitKey(2);

    } catch (cv_bridge::Exception& e) {
      
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
}