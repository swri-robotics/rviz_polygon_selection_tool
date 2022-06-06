cmake_minimum_required(VERSION 3.5)
project(rviz_polygon_selection_tool)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)

# Include the 'rosidl_get_typesupport_target' target on distros earlier than Galactic
if(DEFINED ENV{ROS_DISTRO})
  if("$ENV{ROS_DISTRO}" STREQUAL "foxy")
    include(cmake/macros.cmake)
  endif()
endif()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "srv/GetSelection.srv"
  DEPENDENCIES
  geometry_msgs)

add_library(${PROJECT_NAME}_plugin SHARED src/rviz_polygon_selection_tool.cpp)
ament_target_dependencies(${PROJECT_NAME}_plugin rviz_common pluginlib)
rosidl_get_typesupport_target(msgs_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_plugin ${msgs_target})

# Install the library
install(TARGETS ${PROJECT_NAME}_plugin EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)

ament_export_dependencies(geometry_msgs rosidl_default_runtime)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
ament_package()
