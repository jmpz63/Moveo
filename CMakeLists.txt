cmake_minimum_required(VERSION 3.8)
project(arduino_arm_hardware)

# Ensure modern C++ standards are used
if(CMAKE_COMPILER_ID STREQUAL "GNU")
  # Add -Wall and -Wextra for more warnings during compilation
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

# Find ROS 2 packages that your Python hardware interface depends on
# Even though it's a Python package, it needs to find these for the plugin mechanism
# and for the data files it installs (like URDF, config, launch).
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED) # For Python ROS client library
find_package(hardware_interface REQUIRED) # For ros2_control hardware interface base classes
find_package(controller_manager_msgs REQUIRED) # For messages related to controller manager
find_package(python3_setuptools REQUIRED) # For installing Python packages
find_package(ament_python REQUIRED) # For ament_python build type

# Add include directories (if you had C++ headers, not strictly needed for this Python-only plugin)
# include_directories(
#   include
# )

# Install the Python package
# This command tells ament_python to install the Python module
# located in the 'arduino_arm_hardware' subdirectory.
ament_python_install_package(${PROJECT_NAME})

# Install data files (URDF, config, launch, plugin XML)
# These files are needed at runtime by ROS 2
install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    arduino_arm_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Mark the package as an ament package
ament_package()
