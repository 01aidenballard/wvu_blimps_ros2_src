cmake_minimum_required(VERSION 3.8)
project(blimp_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EscInput.msg"
  "msg/CameraCoord.msg"
  "srv/GetCamera.srv"
)

ament_export_dependencies(rosidl_default_runtime)


ament_package()
