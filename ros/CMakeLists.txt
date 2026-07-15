cmake_minimum_required(VERSION 3.20)
project(robot_model_server_ros VERSION 0.1.0)

include(ccws_set_cmake_flags OPTIONAL RESULT_VARIABLE CCWS_ON)
if(NOT CCWS_ON)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  set(CMAKE_VERBOSE_MAKEFILE ON)

  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_model_server_core REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(${PROJECT_NAME}
  src/robot_model_server_ros.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(${PROJECT_NAME} PRIVATE
  builtin_interfaces::builtin_interfaces
  Eigen3::Eigen
  geometry_msgs::geometry_msgs
  rcl_interfaces::rcl_interfaces
  rclcpp::rclcpp
  robot_model_server_core::robot_model_server_core
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  tf2_ros::tf2_ros
  visualization_msgs::visualization_msgs
)

install(
  TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
