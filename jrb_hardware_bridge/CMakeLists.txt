cmake_minimum_required(VERSION 3.5)
project(jrb_hardware_bridge)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(gpio_node src/gpio_node.cpp)
ament_target_dependencies(gpio_node rclcpp std_msgs serial)

# Install Cpp executables
install(TARGETS
  gpio_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
# install(PROGRAMS
#   scripts/py_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()