cmake_minimum_required(VERSION 3.8)
project(sensors_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(blimp_interfaces REQUIRED)
#find_package(vector REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(balloon_detect_cpp src/balloon_detect_node.cpp)
ament_target_dependencies(balloon_detect_cpp rclcpp blimp_interfaces OpenCV) 

install(TARGETS
  balloon_detect_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
