cmake_minimum_required(VERSION 3.5)
project(moveo2_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial REQUIRED)

add_library(moveo2_plugins SHARED src/moveo2_hardware_interface.cpp src/moveo2_serial_port.cpp)


target_include_directories(
  moveo2_plugins
  PRIVATE
  include
)


ament_target_dependencies(
  moveo2_plugins
  hardware_interface
  controller_manager
  serial
  rclcpp
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface moveo2_plugins.xml)




install(
  TARGETS moveo2_plugins
  DESTINATION lib
)



ament_export_libraries(
  moveo2_plugins
)


ament_package()
