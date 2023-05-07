cmake_minimum_required(VERSION 3.5)
project(jrb_actuators)

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
find_package(sensor_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(teleop_actuators_joy src/teleop_actuators_joy.cpp)
ament_target_dependencies(teleop_actuators_joy rclcpp std_msgs sensor_msgs)

# Install Cpp executables
install(TARGETS
  teleop_actuators_joy
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/arm_teleop.py
  ${PROJECT_NAME}/actuators.py
  ${PROJECT_NAME}/arm_action_server.py
  ${PROJECT_NAME}/generic_action_server.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()