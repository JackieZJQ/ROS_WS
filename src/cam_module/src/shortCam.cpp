#include "shortCam.hpp"

shortCam::shortCam(const std::string &name):Node(name) {

    // 短焦相机发布
    image_msg_publisher = this->create_publisher<sensor_msgs::msg::Image>("short_focal_image", 10);

    // 相机参数
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G')); 
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480); 

    // 打开相机设备
    int deviceID = 0;
    cap.open(deviceID);

    if (!cap.isOpened()) RCLCPP_INFO(this->get_logger(), "---无法打开短焦摄像机---");
    else RCLCPP_INFO(this->get_logger(), "---短焦摄像机初始化成功---");

    // 打开单独线陈读取摄像机
    runOpencam = std::make_shared<std::thread>(std::bind(&shortCam::readImage, this));
    runOpencam->detach();
    
    // Publish
    runPubcam = std::make_shared<std::thread>(std::bind(&shortCam::publishImage, this));
    runPubcam->join();
    
    // 定时器 Publish
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TopicPublisher01::timer_callback, this));
}

shortCam::~shortCam() {
    
    cap.release();

    // cv::destroyAllWindows();
}

void shortCam::readImage() {

    // cv::namedWindow("Short Focal Camera", 1);

    while(rclcpp::ok()) {
        
        cap.read(image);

        // if (!image.empty()) cv::imshow("Short Focal Camera", image); 

        cv::waitKey(2);
    }
}

void shortCam::publishImage() {

    while(rclcpp::ok()) {

        try {

            if (!image.empty()) {
                
                RCLCPP_INFO_ONCE(this->get_logger(), "---Image Publishing---");
                auto mmsg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, image).toImageMsg(); 
                image_msg_publisher->publish(*mmsg);
            } else RCLCPP_WARN_ONCE(this->get_logger(), "---Image Empty---");
            
        } catch (cv_bridge::Exception& e) {
            
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::waitKey(2);
    }   
}