cmake_minimum_required(VERSION 3.5)
project(cam_module)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})

# 添加源文件， 生成库
add_library(camModule SHARED
  src/camSyn.cpp  
  src/longCam.cpp 
  src/shortCam.cpp 
)

# ROS2 自带库链接
set(BASE_DEPENDENCY_LIST 
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

ament_target_dependencies(camModule
  ${BASE_DEPENDENCY_LIST}
)

# 第三方库 链接
target_link_libraries(camModule
  ${OpenCV_LIBS}
)

# 添加头文件地址
target_include_directories(camModule PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(camOpen src/camOpen.cpp)

# 链接自己的库与可执行文件
target_link_libraries(camOpen camModule)

ament_package()

#安装库文件
install(TARGETS
  camModule
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#安装可执行文件
install(TARGETS 
  camOpen 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)






