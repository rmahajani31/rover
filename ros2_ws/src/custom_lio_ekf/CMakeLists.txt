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

add_library(ekf_state
  src/ekf_state.cpp
)

add_library(ekf_parameters
  src/ekf_parameters.cpp
)

target_include_directories(ekf_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(ekf_parameters PUBLIC
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

install(
  TARGETS
    ekf_state
    ekf_parameters
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
ament_export_libraries(ekf_state ekf_parameters)
ament_export_dependencies(Eigen3)

ament_package()
