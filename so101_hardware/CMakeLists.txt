cmake_minimum_required(VERSION 3.8)
project(so101_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(joint_limits REQUIRED)


add_library(so101_hardware SHARED
            src/SO101System.cpp)

add_library(sts_protocol STATIC
            src/sts_protocol.cpp)

add_library(serial_port STATIC
            src/serial_port.cpp)

add_library(sts_bus STATIC
            src/sts_bus.cpp)

target_compile_features(so101_hardware PUBLIC cxx_std_17)
target_compile_features(sts_protocol PUBLIC cxx_std_17)
target_compile_features(serial_port PUBLIC cxx_std_17)
target_compile_features(sts_bus PUBLIC cxx_std_17)

set_target_properties(sts_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(serial_port PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(sts_bus PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(so101_hardware PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(sts_protocol PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(serial_port PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(sts_bus PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>)


target_link_libraries(so101_hardware PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  joint_limits::joint_limits
)

target_link_libraries(sts_bus PRIVATE serial_port sts_protocol)
target_link_libraries(so101_hardware PRIVATE sts_bus sts_protocol)
target_link_options(so101_hardware PRIVATE -Wl,--no-undefined)

pluginlib_export_plugin_description_file(hardware_interface so101_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include/so101_hardware
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test test/test.cpp)

  ament_add_gtest(test_sts_protocol test/test_sts_protocol.cpp)
  target_link_libraries(test_sts_protocol sts_protocol)

  ament_add_gtest(test_serial_port test/test_serial_port.cpp)
  target_link_libraries(test_serial_port serial_port)

  ament_add_gtest(test_sts_bus test/test_sts_bus.cpp)
  target_link_libraries(test_sts_bus sts_bus)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface joint_limits pluginlib rclcpp rclcpp_lifecycle)

ament_package()
