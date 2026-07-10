cmake_minimum_required(VERSION 3.20)
project(robot_model_server_tests VERSION 0.1.0 LANGUAGES CXX)

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_find_gtest()

  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)

  add_executable(test_two_links_fixed_joint test/test_two_links_fixed_joint.cpp)
  target_include_directories(test_two_links_fixed_joint PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(test_two_links_fixed_joint PRIVATE ${GTEST_LIBRARIES} rclcpp::rclcpp tf2_ros::tf2_ros)
  add_launch_test(test/two_links_fixed_joint-launch.py
    ARGS "test_exe:=$<TARGET_FILE:test_two_links_fixed_joint>")

  add_executable(test_two_links_fixed_joint_prefix test/test_two_links_fixed_joint_prefix.cpp)
  target_include_directories(test_two_links_fixed_joint_prefix PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(test_two_links_fixed_joint_prefix PRIVATE ${GTEST_LIBRARIES} rclcpp::rclcpp tf2_ros::tf2_ros)
  add_launch_test(test/two_links_fixed_joint_prefix-launch.py
    ARGS "test_exe:=$<TARGET_FILE:test_two_links_fixed_joint_prefix>")

  add_executable(test_two_links_moving_joint test/test_two_links_moving_joint.cpp)
  target_include_directories(test_two_links_moving_joint PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(test_two_links_moving_joint PRIVATE ${GTEST_LIBRARIES} rclcpp::rclcpp sensor_msgs::sensor_msgs tf2_ros::tf2_ros)
  add_launch_test(test/two_links_moving_joint-launch.py
    ARGS "test_exe:=$<TARGET_FILE:test_two_links_moving_joint>")
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
