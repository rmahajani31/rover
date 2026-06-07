cmake_minimum_required(VERSION 3.8)
project(livox_cloud_to_scan)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(livox_cloud_to_scan_projector
  src/cloud_projector.cpp
)

target_include_directories(livox_cloud_to_scan_projector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(livox_cloud_to_scan_projector
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

add_executable(livox_cloud_to_scan_node
  src/livox_cloud_to_scan_node.cpp
)

target_link_libraries(livox_cloud_to_scan_node
  livox_cloud_to_scan_projector
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  tf2::tf2
  tf2_ros::tf2_ros
)

add_executable(livox_custom_msg_to_pointcloud2_node
  src/livox_custom_msg_to_pointcloud2_node.cpp
)

ament_target_dependencies(livox_custom_msg_to_pointcloud2_node
  livox_ros_driver2
  rclcpp
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_cloud_projector
    test/cloud_projector_test.cpp
  )

  if(TARGET test_cloud_projector)
    target_include_directories(test_cloud_projector PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_link_libraries(test_cloud_projector
      livox_cloud_to_scan_projector
      ${geometry_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      sensor_msgs::sensor_msgs_library
      tf2::tf2
      tf2_geometry_msgs::tf2_geometry_msgs
    )
  endif()
endif()

install(
  TARGETS
    livox_cloud_to_scan_projector
    livox_cloud_to_scan_node
    livox_custom_msg_to_pointcloud2_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  livox_ros_driver2
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_package()
