cmake_minimum_required(VERSION 3.5)

project(jrb_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/AdaptativePIDConfig.msg"
  "msg/MotionBoardDataFrame.msg"
  "msg/MotionConfig.msg"
  "msg/MotionSpeedCommand.msg"
  "msg/OdometryTicks.msg"
  "msg/PIDConfig.msg"
  "msg/PIDState.msg"
  "msg/SampleDetected.msg"
  "msg/SampleDetectedArray.msg"
  "msg/ServoAngle.msg"
  "msg/ServoConfig.msg"
  "msg/ServoID.msg"
  "msg/ServoGenericCommand.msg"
  "msg/ServoGenericRead.msg"
  "msg/ServoGenericReadResponse.msg"
  "msg/ServoStatus.msg"
  "msg/SliderPosition.msg"
  "msg/SliderConfig.msg"
  "msg/SliderStatus.msg"
  "msg/StackSample.msg"
  "msg/BoolArray.msg"
  "msg/ArmStatus.msg"
  "action/GoToPose.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
