cmake_minimum_required(VERSION 3.8)
project(sensors_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-class-memaccess)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(blimp_interfaces REQUIRED)
#find_package(vector REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# detect_node not needed any more (hopefully)
#add_executable(detect_cpp src/detect_node.cpp)
#ament_target_dependencies(detect_cpp rclcpp blimp_interfaces OpenCV sensor_msgs)

add_executable(detect_cpp src/detect_client.cpp)
ament_target_dependencies(detect_cpp 
  rclcpp 
  blimp_interfaces 
  OpenCV 
  sensor_msgs
)

add_executable(balloon_server src/balloon_server.cpp)
ament_target_dependencies(balloon_server
  rclcpp
  blimp_interfaces
  OpenCV
)

add_executable(goal_server src/goal_server.cpp)
ament_target_dependencies(goal_server
  rclcpp
  blimp_interfaces
  OpenCV
)

add_executable(dynamic_model src/inv_kine.cpp)
ament_target_dependencies(dynamic_model
  rclcpp
  blimp_interfaces
  Eigen3
)

add_executable(F_to_Esc src/force_to_ESC_input.cpp)
ament_target_dependencies(F_to_Esc
  rclcpp
  blimp_interfaces
  Eigen3
)

add_executable(pi_controller src/Balloon_pi.cpp)
ament_target_dependencies(pi_controller
  rclcpp
  blimp_interfaces
)

install(TARGETS
  detect_cpp
  balloon_server
  goal_server
  dynamic_model
  F_to_Esc
  pi_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
