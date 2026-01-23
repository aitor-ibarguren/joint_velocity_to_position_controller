cmake_minimum_required(VERSION 3.16)
project(joint_velocity_to_position_controller)

# Basic compiler warnings and options
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  generate_parameter_library
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
)

# Find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Eigen3 REQUIRED)

# Generate parameter library
find_package(generate_parameter_library REQUIRED)

# Generate parameter library
generate_parameter_library(joint_velocity_to_position_controller_params
  config/joint_velocity_to_position_controller_params.yaml
)

# Create the controller as a shared library
add_library(joint_velocity_to_position_controller SHARED
  src/joint_velocity_to_position_controller.cpp
)

# Specify the C++ standard
target_compile_features(joint_velocity_to_position_controller PUBLIC cxx_std_17)

# Include directories for header files
target_include_directories(joint_velocity_to_position_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link against the necessary dependencies and parameter library
ament_target_dependencies(joint_velocity_to_position_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(joint_velocity_to_position_controller PUBLIC joint_velocity_to_position_controller_params)

# Define the plugin for the controller
pluginlib_export_plugin_description_file(controller_interface joint_velocity_to_position_controller_plugin.xml)

# Install the shared library and headers
install(TARGETS
  joint_velocity_to_position_controller
  joint_velocity_to_position_controller_params
  EXPORT export_joint_velocity_to_position_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Export targets and dependencies
ament_export_targets(export_joint_velocity_to_position_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()