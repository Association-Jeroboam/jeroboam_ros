cmake_minimum_required(VERSION 3.5)
project(jrb_sensors)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(OpenCV REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/cherries_counter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()