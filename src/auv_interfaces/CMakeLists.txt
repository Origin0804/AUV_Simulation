cmake_minimum_required(VERSION 3.8)
project(auv_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DVLData.msg"
  "msg/IMUData.msg"
  "msg/CVModel.msg"
  "msg/MotorsData.msg"
  "msg/PositionData.msg"
  "msg/TotalForce.msg"
  "msg/ObjectsData.msg"
  "msg/DetectedObject.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
