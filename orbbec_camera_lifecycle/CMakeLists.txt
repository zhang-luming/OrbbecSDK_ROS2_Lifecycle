cmake_minimum_required(VERSION 3.8)
project(orbbec_camera_lifecycle)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 从 package.xml 提取版本号，作为 pdLog 的 software_version 上报。
set(PACKAGE_XML_PATH "${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
file(READ "${PACKAGE_XML_PATH}" PACKAGE_XML_CONTENT)
string(REGEX MATCH "<version>([0-9]+\\.[0-9]+\\.[0-9]+)</version>" _ ${PACKAGE_XML_CONTENT})
set(PACKAGE_VERSION ${CMAKE_MATCH_1})
message(STATUS "Package version: ${PACKAGE_VERSION}")
add_definitions("-DORBBEC_LIFECYCLE_VERSION=\"${PACKAGE_VERSION}\"")

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
find_package(pudu-base REQUIRED)
find_package(nvq_tools REQUIRED)

add_executable(orbbec_camera_lifecycle_node src/lifecycle_camera_node.cpp)
target_include_directories(orbbec_camera_lifecycle_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(orbbec_camera_lifecycle_node
  lifecycle_msgs
  nvq_tools
  orbbec_camera
  rclcpp
  rclcpp_components
  rclcpp_lifecycle)
target_link_libraries(orbbec_camera_lifecycle_node yaml-cpp pudu-base::pudu-base)

add_executable(camera_self_check src/camera_self_check.cpp)
target_include_directories(camera_self_check PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(camera_self_check
  ament_index_cpp
  nvq_tools
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  std_srvs)
target_link_libraries(camera_self_check yaml-cpp pudu-base::pudu-base)

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
