cmake_minimum_required(VERSION 3.8)
project(custom_lio_ekf)

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
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Eigen3 REQUIRED)
find_package(custom_imu_propagator REQUIRED)
find_package(custom_scan_to_map_odom REQUIRED)
find_package(custom_ikd_tree_backend REQUIRED)

add_library(ekf_state
  src/ekf_state.cpp
)

add_library(ekf_parameters
  src/ekf_parameters.cpp
)

add_library(imu_prediction
  src/imu_prediction.cpp
)

add_library(covariance_propagation
  src/covariance_propagation.cpp
)

add_library(lidar_residual
  src/lidar_residual.cpp
)

add_library(iterated_lidar_update
  src/iterated_lidar_update.cpp
)

add_library(custom_lio_ekf_diagnostics
  src/diagnostics.cpp
)

add_executable(custom_lio_ekf_node
  src/lio_ekf_node.cpp
)

target_include_directories(ekf_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(ekf_parameters PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(imu_prediction PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(covariance_propagation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(lidar_residual PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(iterated_lidar_update PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(custom_lio_ekf_diagnostics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(custom_lio_ekf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(ekf_state
  Eigen3::Eigen
)

target_link_libraries(ekf_parameters
  ekf_state
  Eigen3::Eigen
)

ament_target_dependencies(imu_prediction
  custom_imu_propagator
)

target_link_libraries(imu_prediction
  ekf_state
  Eigen3::Eigen
)

ament_target_dependencies(covariance_propagation
  custom_imu_propagator
)

target_link_libraries(covariance_propagation
  ekf_parameters
  ekf_state
  imu_prediction
  Eigen3::Eigen
)

ament_target_dependencies(lidar_residual
  custom_scan_to_map_odom
  custom_ikd_tree_backend
)

target_link_libraries(lidar_residual
  ekf_parameters
  ekf_state
  Eigen3::Eigen
)

ament_target_dependencies(iterated_lidar_update
  custom_scan_to_map_odom
  custom_ikd_tree_backend
)

target_link_libraries(iterated_lidar_update
  ekf_parameters
  ekf_state
  lidar_residual
  Eigen3::Eigen
)

ament_target_dependencies(custom_lio_ekf_diagnostics
  builtin_interfaces
  diagnostic_msgs
)

target_link_libraries(custom_lio_ekf_diagnostics
  covariance_propagation
  ekf_state
  iterated_lidar_update
  Eigen3::Eigen
)

ament_target_dependencies(custom_lio_ekf_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
  pcl_ros
  custom_imu_propagator
  custom_scan_to_map_odom
  custom_ikd_tree_backend
)

target_link_libraries(custom_lio_ekf_node
  covariance_propagation
  custom_lio_ekf_diagnostics
  ekf_parameters
  ekf_state
  iterated_lidar_update
  lidar_residual
  custom_ikd_tree_backend
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_diagnostics
    test/test_diagnostics.cpp
  )

  target_link_libraries(test_diagnostics
    custom_lio_ekf_diagnostics
  )
endif()

install(
  TARGETS
    ekf_state
    ekf_parameters
    imu_prediction
    covariance_propagation
    lidar_residual
    iterated_lidar_update
    custom_lio_ekf_diagnostics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS custom_lio_ekf_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(
  ekf_state
  ekf_parameters
  imu_prediction
  covariance_propagation
  lidar_residual
  iterated_lidar_update
  custom_lio_ekf_diagnostics
)
ament_export_dependencies(
  Eigen3
  custom_imu_propagator
  custom_scan_to_map_odom
  custom_ikd_tree_backend
)

ament_package()
