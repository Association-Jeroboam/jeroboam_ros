#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cstdlib>
#include <thread>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <sstream>
#include <vector>

#include "CyphalWrapper.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "jrb_msgs/msg/pid_state.hpp"
#include "jrb_msgs/msg/pump_status.hpp"
#include "jrb_msgs/msg/valve_status.hpp"
#include "jrb_msgs/msg/pid_config.hpp"
#include "jrb_msgs/msg/adaptative_pid_config.hpp"
#include "jrb_msgs/msg/motion_config.hpp"
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sysinfo.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "canard.h"
#include "Heartbeat_1_0.h"
#include "cartesian/State_0_1.h"
#include "cartesian/Twist_0_1.h"
#include "CanProtocol.hpp"
#include "PIDState_0_1.h"
#include "PumpStatus_0_1.h"
#include "ValveStatus_0_1.h"
#include "PIDConfig_0_1.h"
#include "AdaptativePIDConfig_0_1.h"
#include "MotionConfig_0_1.h"
#include "jrb_can_bridge/param_utils.hpp"
#include "jrb_msgs/msg/servo_angle.hpp"
#include "jrb_msgs/msg/servo_config.hpp"
#include "ServoAngle_0_1.h"
#include "ServoConfig_0_1.h"
#include "CanBridge.hpp"

  using namespace std::chrono_literals;


void createSubscriptions(void);
bool check_parameter(char * iface, char * name, size_t n);
void publishReceivedMessage(CanardRxTransfer * transfer);

void print_usage(void) {
    printf("usage:\n\tchyphal_demo can_interface (can0, vcan0...)\n");
}

CyphalWrapper * wrapperPtr = new CyphalWrapper(publishReceivedMessage);
auto bridgePtr  = new CanBridge(wrapperPtr);

std::shared_ptr<CanBridge> canBridge(bridgePtr);
std::shared_ptr<CyphalWrapper> cyphalWrapper(wrapperPtr);


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  cyphalWrapper.get()->init();
  createSubscriptions();


  
  

  // canBridge = std::make_shared<CanBridge>(wrapperPtr);
  std::this_thread::sleep_for(100ms);

  std::this_thread::sleep_for(100ms);
  canBridge.get()->init();
  rclcpp::spin(canBridge);
  rclcpp::shutdown();
  cyphalWrapper.get()->shutdown();

  return 0;
}

void createSubscriptions(void) {
  cyphalWrapper.get()->subscribe(CanardTransferKindMessage,
                                 ROBOT_CURRENT_STATE_ID,
                                 reg_udral_physics_kinematics_cartesian_State_0_1_EXTENT_BYTES_);
  cyphalWrapper.get()->subscribe(CanardTransferKindMessage,
                                 uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
                                 uavcan_node_Heartbeat_1_0_EXTENT_BYTES_);
  cyphalWrapper.get()->subscribe(CanardTransferKindMessage,
                                 MOTION_PID_STATE_ID,
                                 jeroboam_datatypes_actuators_motion_PIDState_0_1_EXTENT_BYTES_);
  cyphalWrapper.get()->subscribe(CanardTransferKindMessage,
                                 ACTION_PUMP_STATUS_ID,
                                 jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_EXTENT_BYTES_);
  cyphalWrapper.get()->subscribe(CanardTransferKindMessage,
                                 ACTION_VALVE_STATUS_ID,
                                 jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_EXTENT_BYTES_);
}

void publishReceivedMessage(CanardRxTransfer * transfer) {
  static uint32_t last_transfer_id =                    0;
  static uint64_t frameCount = 0;
  static uint64_t frameErrorCount = 0;
  switch (transfer->metadata.port_id)
  {
    case ROBOT_CURRENT_STATE_ID:{
        frameCount++;
      if((last_transfer_id +1) % 32 != transfer->metadata.transfer_id) {
          frameErrorCount++;
          printf("Transfer lost! %u %u. rate %.4f\n", last_transfer_id, transfer->metadata.transfer_id, (float)frameErrorCount/(float)frameCount);
      }
      last_transfer_id = transfer->metadata.transfer_id;
      reg_udral_physics_kinematics_cartesian_State_0_1 state;
      reg_udral_physics_kinematics_cartesian_State_0_1_deserialize_(&state,
                                                                  (uint8_t *)transfer->payload,
                                                                  &transfer->payload_size);
      canBridge.get()->publishRobotCurrentState(&state);
      break;
    }
    case MOTION_PID_STATE_ID:{
      jeroboam_datatypes_actuators_motion_PIDState_0_1 pidState;
      jeroboam_datatypes_actuators_motion_PIDState_0_1_deserialize_(&pidState,
                                                                    (uint8_t *)transfer->payload,
                                                                    &transfer->payload_size);
      if(pidState.ID == 0) {
        canBridge.get()->publishLeftPIDState(&pidState);
      } else {
        canBridge.get()->publishRightPIDState(&pidState);
      }
      break;
    }
    case ACTION_PUMP_STATUS_ID:{
      jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 status;
      jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_deserialize_(&status,
                                                                    (uint8_t *)transfer->payload,
                                                                    &transfer->payload_size);
      if(status.status.ID == 0) {
        canBridge.get()->publishLeftPumpStatus(&status);
      } else {
        canBridge.get()->publishRightPumpStatus(&status);
      }
      break;
    }
    case ACTION_VALVE_STATUS_ID:{
      jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 status;
      jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_deserialize_(&status,
                                                                    (uint8_t *)transfer->payload,
                                                                    &transfer->payload_size);
      if(status.status.ID == 0) {
        canBridge.get()->publishLeftValveStatus(&status);
      } else {
        canBridge.get()->publishRightValveStatus(&status);
      }
      break;
    }
    case uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_:
        // Heartbeat
        break;
    default:
      printf("Accepted a non handled CAN message(ID %u)! Please fix me!\n", transfer->metadata.port_id);
      break;
  }
}

bool check_parameter(char * iface, char * name, size_t n) {
	return memcmp(iface, name, n) == 0;
}
