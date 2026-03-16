cmake_minimum_required(VERSION 3.5)
project(leo_bldc)

option(CANDLESDK_BUILD_EXAMPLES "" OFF)
option(CANDLESDK_BUILD_CANDLETOOL "" OFF)
option(MAKE_TESTS "" OFF)
add_subdirectory(CANdle-SDK)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs)
find_package(generate_parameter_library REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

generate_parameter_library(
  bldc_manager_parameters
  src/bldc_manager_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/bldc_manager.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${PROJECT_NAME}
  bldc_manager_parameters  
  candle
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN leo_bldc::BLDCManager
  EXECUTABLE bldc_manager
  EXECUTOR SingleThreadedExecutor
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME} candle
  # EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  generate_parameter_library
  nav_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()