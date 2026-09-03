cmake_minimum_required(VERSION 3.8)
project(orbbec_camera_lifecycle)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(orbbec_camera REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(orbbec_camera_lifecycle_node src/lifecycle_camera_node.cpp)
target_include_directories(orbbec_camera_lifecycle_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(orbbec_camera_lifecycle_node
  lifecycle_msgs
  orbbec_camera
  rclcpp
  rclcpp_components
  rclcpp_lifecycle)
target_link_libraries(orbbec_camera_lifecycle_node yaml-cpp)

add_executable(camera_self_check src/camera_self_check.cpp)
target_include_directories(camera_self_check PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(camera_self_check
  ament_index_cpp
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  std_srvs)
target_link_libraries(camera_self_check yaml-cpp)

install(TARGETS orbbec_camera_lifecycle_node camera_self_check
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
