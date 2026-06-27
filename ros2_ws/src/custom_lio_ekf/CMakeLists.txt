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
find_package(Eigen3 REQUIRED)
find_package(custom_imu_propagator REQUIRED)
find_package(custom_scan_to_map_odom REQUIRED)

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

add_library(diagnostics
  src/diagnostics.cpp
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

target_include_directories(diagnostics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
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
)

target_link_libraries(lidar_residual
  ekf_parameters
  ekf_state
  Eigen3::Eigen
)

ament_target_dependencies(iterated_lidar_update
  custom_scan_to_map_odom
)

target_link_libraries(iterated_lidar_update
  ekf_parameters
  ekf_state
  lidar_residual
  Eigen3::Eigen
)

ament_target_dependencies(diagnostics
  builtin_interfaces
  diagnostic_msgs
)

target_link_libraries(diagnostics
  covariance_propagation
  ekf_state
  iterated_lidar_update
  Eigen3::Eigen
)

install(
  TARGETS
    ekf_state
    ekf_parameters
    imu_prediction
    covariance_propagation
    lidar_residual
    iterated_lidar_update
    diagnostics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
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
  diagnostics
)
ament_export_dependencies(Eigen3 custom_imu_propagator custom_scan_to_map_odom)

ament_package()
