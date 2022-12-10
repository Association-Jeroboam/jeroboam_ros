cmake_minimum_required(VERSION 3.8)
project(jrb_strategy2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

set(library_name ${PROJECT_NAME})

include_directories(
  include
)

set(dependencies
  rclcpp
  behaviortree_cpp_v3
  tf2_ros
  std_msgs
)

add_library(is_match_started_condition SHARED src/nodes/condition/is_match_started_condition.cpp)
list(APPEND plugin_libs is_match_started_condition)

add_library(is_equal_condition SHARED src/nodes/condition/is_equal_condition.cpp)
list(APPEND plugin_libs is_equal_condition)

add_library(is_timer_over_condition SHARED src/nodes/condition/is_timer_over_condition.cpp)
list(APPEND plugin_libs is_timer_over_condition)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(brain_node
  src/brain_node.cpp
)
ament_target_dependencies(brain_node ${dependencies})
# if built statically
# target_link_libraries(brain_node ${plugin_libs})


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS brain_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_export_dependencies(${dependencies})
ament_package()
