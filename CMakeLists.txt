cmake_minimum_required(VERSION 3.5)
project(xela_server_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorFull.msg"
  "msg/SensStream.msg"
  "msg/Taxel.msg"
  "msg/Forces.msg"
  "srv/XelaSensorStream.srv"
  "srv/XelaSensor.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

set(dependencies "std_msgs" "geometry_msgs")

ament_export_dependencies(
    rclpy 
    std_msgs 
    geometry_msgs
    rosidl_runtime_py
)

###########
## Build ##
###########

include_directories(include ${Boost_INCLUDE_DIRS})

#############
## Install ##
#############

install(PROGRAMS
  scripts/xela_server
  scripts/xela_service
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
