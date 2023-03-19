cmake_minimum_required(VERSION 3.5)
project(jrb_screen)

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
find_package(raylib 3.0 REQUIRED) 

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(screen_manager src/screen_manager.cpp)
ament_target_dependencies(screen_manager rclcpp std_msgs)
target_link_libraries(screen_manager raylib)

# Install Cpp executables
install(TARGETS
  screen_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
# install(PROGRAMS
#   scripts/py_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY
  assets
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()