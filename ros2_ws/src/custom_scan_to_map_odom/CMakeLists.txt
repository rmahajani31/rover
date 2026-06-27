cmake_minimum_required(VERSION 3.8)
project(custom_scan_to_map_odom)

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
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree)
find_package(Eigen3 REQUIRED)
find_package(custom_imu_propagator REQUIRED)

add_library(se3_utils
  src/se3_utils.cpp
)

add_library(ros_conversions
  src/ros_conversions.cpp
)

add_library(plane_fitter
  src/plane_fitter.cpp
)

add_library(local_map
  src/local_map.cpp
)

add_library(local_map_manager
  src/local_map_manager.cpp
)

add_library(scan_to_map_optimizer
  src/scan_to_map_optimizer.cpp
)

add_library(diagnostics
  src/diagnostics.cpp
)

add_executable(custom_scan_to_map_odom_node
  src/scan_to_map_node.cpp
)

target_include_directories(se3_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(ros_conversions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(plane_fitter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(local_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(local_map_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(scan_to_map_optimizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(diagnostics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(custom_scan_to_map_odom_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(se3_utils
  geometry_msgs
)

ament_target_dependencies(ros_conversions
  sensor_msgs
  std_msgs
  pcl_conversions
)

target_link_libraries(ros_conversions
  ${PCL_LIBRARIES}
)

target_link_libraries(local_map
  ros_conversions
  ${PCL_LIBRARIES}
)

target_link_libraries(local_map_manager
  local_map
)

target_link_libraries(scan_to_map_optimizer
  local_map
  plane_fitter
  ros_conversions
  se3_utils
  ${PCL_LIBRARIES}
)

ament_target_dependencies(diagnostics
  builtin_interfaces
  diagnostic_msgs
)

target_link_libraries(diagnostics
  scan_to_map_optimizer
)

ament_target_dependencies(custom_scan_to_map_odom_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  diagnostic_msgs
  custom_imu_propagator
  tf2
  tf2_ros
  tf2_eigen
)

target_link_libraries(custom_scan_to_map_odom_node
  diagnostics
  local_map
  local_map_manager
  ros_conversions
  scan_to_map_optimizer
  se3_utils
  ${PCL_LIBRARIES}
)

install(
  TARGETS
    se3_utils
    ros_conversions
    plane_fitter
    local_map
    local_map_manager
    scan_to_map_optimizer
    diagnostics
  EXPORT export_custom_scan_to_map_odom
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS custom_scan_to_map_odom_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_se3_utils test/test_se3_utils.cpp)
  target_link_libraries(test_se3_utils se3_utils)

  ament_add_gtest(test_plane_fitter test/test_plane_fitter.cpp)
  target_link_libraries(test_plane_fitter plane_fitter)

  ament_add_gtest(test_local_map_manager test/test_local_map_manager.cpp)
  target_link_libraries(test_local_map_manager local_map_manager)
endif()

ament_export_include_directories(include)
ament_export_targets(export_custom_scan_to_map_odom HAS_LIBRARY_TARGET)
ament_export_libraries(
  se3_utils
  ros_conversions
  plane_fitter
  local_map
  local_map_manager
  scan_to_map_optimizer
  diagnostics
)
ament_export_dependencies(
  geometry_msgs
  sensor_msgs
  std_msgs
  builtin_interfaces
  diagnostic_msgs
  pcl_conversions
  PCL
  Eigen3
)

ament_package()
