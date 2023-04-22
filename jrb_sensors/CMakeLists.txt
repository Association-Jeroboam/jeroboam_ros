cmake_minimum_required(VERSION 3.5)
project(jrb_sensors)

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
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(visualization_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED) 
find_package(jrb_msgs REQUIRED) 
find_package(OpenCV REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(sample_detector src/sample_detector.cpp)
ament_target_dependencies(sample_detector OpenCV rclcpp tf2_ros cv_bridge image_transport std_msgs geometry_msgs visualization_msgs sensor_msgs jrb_msgs)

# Install Cpp executables
install(TARGETS
  sample_detector
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/obstacle_detector.py
  ${PROJECT_NAME}/sample_detector.py
  ${PROJECT_NAME}/cherries_counter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()