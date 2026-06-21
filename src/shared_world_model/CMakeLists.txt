cmake_minimum_required(VERSION 3.8)
project(shared_world_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}

)

add_executable(shared_world_model_node src/main.cpp)

ament_target_dependencies(shared_world_model_node rclcpp Eigen3)

install(TARGETS
  shared_world_model_node
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()