cmake_minimum_required(VERSION 3.8)
project(custom_imu_propagator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(imu_propagator
  src/imu_propagator.cpp
)

add_executable(custom_imu_propagator_node
  src/imu_propagator_node.cpp
)

target_include_directories(imu_propagator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(custom_imu_propagator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(imu_propagator
  rclcpp
)

ament_target_dependencies(custom_imu_propagator_node
  builtin_interfaces
  diagnostic_msgs
  rclcpp
  sensor_msgs
)

target_link_libraries(imu_propagator
  Eigen3::Eigen
)

target_link_libraries(custom_imu_propagator_node
  imu_propagator
  Eigen3::Eigen
)

install(
  TARGETS imu_propagator
  EXPORT export_imu_propagator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS custom_imu_propagator_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_delta_quaternion test/test_delta_quaternion.cpp)
  target_link_libraries(test_delta_quaternion imu_propagator)
  ament_target_dependencies(test_delta_quaternion rclcpp)

  ament_add_gtest(test_imu_interval_extraction test/test_imu_interval_extraction.cpp)
  target_link_libraries(test_imu_interval_extraction imu_propagator)
  ament_target_dependencies(test_imu_interval_extraction rclcpp)

  ament_add_gtest(test_rotation_integration test/test_rotation_integration.cpp)
  target_link_libraries(test_rotation_integration imu_propagator)
  ament_target_dependencies(test_rotation_integration rclcpp)
endif()

ament_export_include_directories(include)
ament_export_targets(export_imu_propagator HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3)

ament_package()
