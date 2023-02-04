cmake_minimum_required(VERSION 3.8)
project(jrb_can_bridge)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jrb_msgs REQUIRED)

include_directories(CanProtocol/Cyphal/libcanard/libcanard)
include_directories(CanProtocol/)
include_directories(CanProtocol/Cyphal/includes/jeroboam_datatypes)
include_directories(CanProtocol/Cyphal/includes/jeroboam_datatypes/actuators)
include_directories(CanProtocol/Cyphal/includes/jeroboam_datatypes/actuators/motion)
include_directories(CanProtocol/Cyphal/includes/jeroboam_datatypes/actuators/pneumatics)
include_directories(CanProtocol/Cyphal/includes/jeroboam_datatypes/actuators/servo)
include_directories(CanProtocol/Cyphal/includes/jeroboam_datatypes/sensors)
include_directories(CanProtocol/Cyphal/includes/jeroboam_datatypes/sensors/odometry)
include_directories(CanProtocol/Cyphal/includes/)
include_directories(CanProtocol/Cyphal/includes/nunavut)
include_directories(CanProtocol/Cyphal/includes/nunavut/support)
include_directories(CanProtocol/Cyphal/includes/reg)
include_directories(CanProtocol/Cyphal/includes/reg/udral)
include_directories(CanProtocol/Cyphal/includes/reg/udral/physics)
include_directories(CanProtocol/Cyphal/includes/reg/udral/physics/acoustics)
include_directories(CanProtocol/Cyphal/includes/reg/udral/physics/dynamics)
include_directories(CanProtocol/Cyphal/includes/reg/udral/physics/dynamics/rotation)
include_directories(CanProtocol/Cyphal/includes/reg/udral/physics/dynamics/translation)
include_directories(CanProtocol/Cyphal/includes/reg/udral/physics/electricity)
include_directories(CanProtocol/Cyphal/includes/reg/udral/physics/kinematics)
include_directories(CanProtocol/Cyphal/includes/reg/udral/physics/kinematics/cartesian)
include_directories(CanProtocol/Cyphal/includes/reg/udral/physics/kinematics/geodetic)
include_directories(CanProtocol/Cyphal/includes/reg/udral/physics/kinematics/rotation)
include_directories(CanProtocol/Cyphal/includes/reg/udral/physics/kinematics/translation)
include_directories(CanProtocol/Cyphal/includes/reg/udral/physics/optics)
include_directories(CanProtocol/Cyphal/includes/reg/udral/physics/thermodynamics)
include_directories(CanProtocol/Cyphal/includes/reg/udral/physics/time)
include_directories(CanProtocol/Cyphal/includes/reg/udral/service)
include_directories(CanProtocol/Cyphal/includes/reg/udral/service/actuator)
include_directories(CanProtocol/Cyphal/includes/reg/udral/service/actuator/common)
include_directories(CanProtocol/Cyphal/includes/reg/udral/service/actuator/esc)
include_directories(CanProtocol/Cyphal/includes/reg/udral/service/actuator/servo)
include_directories(CanProtocol/Cyphal/includes/reg/udral/service/battery)
include_directories(CanProtocol/Cyphal/includes/reg/udral/service/common)
include_directories(CanProtocol/Cyphal/includes/reg/udral/service/sensor)
include_directories(CanProtocol/Cyphal/includes/uavcan)
include_directories(CanProtocol/Cyphal/includes/uavcan/_register)
include_directories(CanProtocol/Cyphal/includes/uavcan/diagnostic)
include_directories(CanProtocol/Cyphal/includes/uavcan/file)
include_directories(CanProtocol/Cyphal/includes/uavcan/internet/udp)
include_directories(CanProtocol/Cyphal/includes/uavcan/metatransport)
include_directories(CanProtocol/Cyphal/includes/uavcan/metatransport/can)
include_directories(CanProtocol/Cyphal/includes/uavcan/metatransport/ethernet)
include_directories(CanProtocol/Cyphal/includes/uavcan/metatransport/serial)
include_directories(CanProtocol/Cyphal/includes/uavcan/metatransport/udp)
include_directories(CanProtocol/Cyphal/includes/uavcan/node)
include_directories(CanProtocol/Cyphal/includes/uavcan/pnp)
include_directories(CanProtocol/Cyphal/includes/uavcan/primitive)
include_directories(CanProtocol/Cyphal/includes/uavcan/primitive/array)
include_directories(CanProtocol/Cyphal/includes/uavcan/primitive/scalar)
include_directories(CanProtocol/Cyphal/includes/uavcan/si)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/acceleration)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/angle)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/angular_acceleration)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/angular_velocity)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/duration)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/electric_charge)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/electric_current)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/energy)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/force)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/frequency)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/length)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/magnetic_field_strength)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/mass)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/power)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/pressure)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/temperature)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/torque)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/velocity)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/voltage)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/volume)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/sample/volumetric_flow_rate)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/acceleration)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/angle)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/angular_acceleration)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/angular_velocity)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/duration)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/electric_charge)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/electric_current)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/energy)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/force)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/frequency)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/length)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/magnetic_field_strength)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/mass)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/power)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/pressure)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/temperature)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/torque)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/velocity)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/voltage)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/volume)
include_directories(CanProtocol/Cyphal/includes/uavcan/si/unit/volumetric_flow_rate)
include_directories(CanProtocol/Cyphal/includes/uavcan/time)

# add_compile_definitions(CANARD_ASSERT (void))

include_directories(
  include
  include/jrb_can_bridge
)

add_executable(jrb_can_bridge
  src/jrb_can_bridge.cpp
  src/CanBridgeTx.cpp
  src/CanBridgeRx.cpp
  src/CanBridge.cpp
  src/LinuxCan.cpp
  src/param_utils.cpp
  CanProtocol/Cyphal/libcanard/libcanard/canard.c
)
ament_target_dependencies(jrb_can_bridge rclcpp std_msgs nav_msgs geometry_msgs jrb_msgs)
target_include_directories(jrb_can_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
#target_compile_features(jrb_can_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  jrb_can_bridge
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
