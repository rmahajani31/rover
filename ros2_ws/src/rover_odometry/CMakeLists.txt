cmake_minimum_required(VERSION 3.8)
project(rover_odometry)

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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(rover_odometry_core
  src/odometry_node.cpp
  src/pinpoint_i2c.cpp
)

target_include_directories(rover_odometry_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(rover_odometry_core
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
)

add_executable(odometry_node
  src/main.cpp
)

target_link_libraries(odometry_node
  rover_odometry_core
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
)

install(
  TARGETS rover_odometry_core odometry_node
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

  ament_add_gtest(test_odometry_node
    test/odometry_node_test.cpp
  )

  if(TARGET test_odometry_node)
    target_include_directories(test_odometry_node PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_link_libraries(test_odometry_node
      rover_odometry_core
      ${geometry_msgs_TARGETS}
      ${nav_msgs_TARGETS}
      rclcpp::rclcpp
      tf2::tf2
      tf2_ros::tf2_ros
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs nav_msgs rclcpp tf2 tf2_ros)

ament_package()
