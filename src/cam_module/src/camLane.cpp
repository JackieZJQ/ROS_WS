#include "camLane.hpp"

camLane::camLane(const std::string &name):Node(name) {

    RCLCPP_INFO(this->get_logger(), "---车道线检测模块初始化---");

    // shortImagecallbackGroup = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    shortImagesub = this->create_subscription<sensor_msgs::msg::Image>("short_focal_image", 10, std::bind(&camLane::shortImagecallback, this, _1));

    cv::namedWindow("Test Window", 1);

    cudaAvailable = torch::cuda::is_available();
    if (cudaAvailable) RCLCPP_INFO(this->get_logger(), "---CUDA available! Predicting on GPU---");

    // 设置推理设备
    torch::DeviceType device_type;
    
    if (cudaAvailable) device_type = torch::kCUDA;
    else device_type = torch::kCPU;

    torch::Device device(device_type);

    // 加载模型
    std::string networkName = "/home/ubuntu/JiaqiZhang/ROS_WS/model/culane_model.pt";
    std::cout << "Loading " << networkName << std::endl;
    module = torch::jit::load(networkName, device);
    std::cout << "Loaded" << std::endl;
}

camLane::~camLane() {

    cv::destroyAllWindows();
}

void camLane::shortImagecallback(const sensor_msgs::msg::Image::SharedPtr image) {

    try {
        
        auto cvPtr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

        cv::imshow("Test Window", cvPtr->image);
        cv::waitKey(2);
        // camLane::forward(cvPtr->image);



    } catch (cv_bridge::Exception& e) {
      
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
}

void camLane::forward(const cv::Mat &_image) {

    if (_image.empty()) return;

    // 设置推理设备
    torch::DeviceType device_type;
    
    if (cudaAvailable) device_type = torch::kCUDA;
    else device_type = torch::kCPU;

    torch::Device device(device_type);

    // 裁剪图片
    cv::Mat image = _image(cv::Range(cut_height, ori_img_h), cv::Range(0, ori_img_w));

    // std::cout << image_src.size() << std::endl;

    cv::resize(image, image, cv::Size(train_width, train_height), 0, 0, cv::INTER_CUBIC);

    cv::Mat img;
    image.convertTo(image, -1, 1.f/255.f, 0);

    //TO-DO？
    //Normalize
    
    std::vector<int64_t> dims = {train_width, train_height, 3, 1};
    torch::Tensor img_var = torch::from_blob(image.data, dims, torch::kByte).to(device);
    img_var = img_var.permute({3, 2, 0, 1});
    img_var = img_var.to(torch::kFloat32);

    std::vector<torch::jit::IValue> inputs = {img_var};
    auto outputs = module.forward(inputs).toGenericDict();
    
    std::vector<std::vector<std::pair<int, int>>> coords;
    // coords = pred2coords(outputs);
    

    // std::cout << image.type() << std::endl;
    // std::cout << image_dst.size() << std::endl;

    cv::imshow("Test Window", image);
    cv::waitKey(2);
}

// std::vector<std::vector<std::pair<int, int>>> camLane::pred2coords(const c10::impl::GenericDict &preds) {

    // toTensor
    // auto loc_row = preds.at("loc_row").toTensor();
    // auto loc_col = preds.at("loc_col").toTensor();
    // auto exist_row = preds.at("exist_row").toTensor();
    // auto exist_col = preds.at("exist_row").toTensor();

    // batch_size_row, num_grid_row, num_cls_row, num_lane_row 
    // int batch_size_row =  loc_row.sizes()[0];
    // int num_grid_row = loc_row.sizes()[1]; 
    // int num_cls_row = loc_row.sizes()[2];
    // int num_lane_row = loc_row.sizes()[3];

    // batch_size_col, num_grid_col, num_cls_col, num_lane_col
    // int batch_size_col = loc_col.sizes()[0];
    // int num_grid_col = loc_col.sizes()[1]; 
    // int num_cls_col = loc_col.sizes()[2];
    // int num_lane_col = loc_col.sizes()[3];

    // auto max_indices_row = loc_row.argmax(1);
    // auto valid_row = exist_row.argmax(1);

    // std::cout << valid_row << std::endl;

    // auto max_indices_col = loc_col.argmax(1);
    // auto valid_col = exist_col.argmax(1);

    // initlize result
    // std::vector<std::vector<std::pair<int, int>>> res;
    // int row_lane_idx[2] = {1, 2};
    // int col_lane_idx[2] = {0, 3};
    
    // std::cout << valid_row.index({0, torch::indexing::Slice(torch::indexing::None, torch::indexing::None), 1}).sum() << std::endl;
    // std::cout << valid_row.sizes()[1] << std::endl;

    // for (int i: row_lane_idx) {
    //     std::vector<std::pair<int, int>> tmp;
    //     if (valid_row[0, :, i].sum())

    // }

    // for (int i: col_lane_idx) {

    // }

    // return res;
// }

