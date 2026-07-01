cmake_minimum_required(VERSION 3.8)
project(custom_ikd_tree_backend)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree)
find_package(Eigen3 REQUIRED)

add_library(map_point
  src/map_point.cpp
)

add_library(bounding_box
  src/bounding_box.cpp
)

add_library(voxel_key
  src/voxel_key.cpp
)

add_library(pcl_rebuild_backend
  src/pcl_rebuild_backend.cpp
)

add_library(backend_profiler
  src/backend_profiler.cpp
)

add_library(voxel_hash_backend
  src/voxel_hash_backend.cpp
)

add_library(kd_tree_node
  src/kd_tree_node.cpp
)

add_library(incremental_kd_tree
  src/incremental_kd_tree.cpp
)

add_library(ikd_tree_backend
  src/ikd_tree_backend.cpp
)

add_library(${PROJECT_NAME}
  src/backend_profiler.cpp
  src/bounding_box.cpp
  src/ikd_tree_backend.cpp
  src/incremental_kd_tree.cpp
  src/kd_tree_node.cpp
  src/map_point.cpp
  src/pcl_rebuild_backend.cpp
  src/voxel_hash_backend.cpp
  src/voxel_key.cpp
)

target_include_directories(map_point PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(bounding_box PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(voxel_key PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(pcl_rebuild_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(backend_profiler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(voxel_hash_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(kd_tree_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(incremental_kd_tree PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(ikd_tree_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(map_point
  Eigen3::Eigen
)

target_link_libraries(bounding_box
  Eigen3::Eigen
)

target_link_libraries(voxel_key
  Eigen3::Eigen
)

target_link_libraries(pcl_rebuild_backend
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

target_link_libraries(voxel_hash_backend
  backend_profiler
  voxel_key
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

target_link_libraries(kd_tree_node
  bounding_box
  Eigen3::Eigen
)

target_link_libraries(incremental_kd_tree
  bounding_box
  kd_tree_node
  Eigen3::Eigen
)

target_link_libraries(ikd_tree_backend
  backend_profiler
  bounding_box
  incremental_kd_tree
  voxel_key
  Eigen3::Eigen
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

install(
  TARGETS
    ${PROJECT_NAME}
    map_point
    bounding_box
    voxel_key
    pcl_rebuild_backend
    backend_profiler
    voxel_hash_backend
    kd_tree_node
    incremental_kd_tree
    ikd_tree_backend
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(PCL Eigen3)

ament_package()
