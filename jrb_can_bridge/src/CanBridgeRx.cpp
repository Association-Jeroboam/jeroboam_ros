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
#include "CanardInterface.hpp"
#include "LinuxCan.hpp"
#include "CanBridgeRx.hpp"
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
#include "jrb_can_bridge.hpp"


const uint32_t CAN_EXT_ID_MASK = (1 <<29) - 1;

void* checkRxMsg(void*);
void publishReceivedMessage(CanardRxTransfer * transfer);

pthread_t rxThread;



void RxThread::CanBridgeInitRxThread() {
    pthread_create(&rxThread, NULL, &checkRxMsg, NULL);
}

void* RxThread::CanBridgeDeinitRxThread() {
    void* retval;
    pthread_cancel(rxThread);
    pthread_join(rxThread, &retval);
    return retval;
}

void* checkRxMsg(void*) {
//  auto lastReceiveTS = std::chrono::high_resolution_clock::now();
//  uint32_t count = 0;
  while (true) {
    if(canIFace !=0) {
      struct can_frame rx_frame;
      //this is blocking
      int nbytes = read(canIFace, &rx_frame, sizeof(struct can_frame));
//        auto receiveTS = std::chrono::high_resolution_clock::now();
//        auto dt = receiveTS - lastReceiveTS;
//        lastReceiveTS = receiveTS;

      if (nbytes >= 0) {
        const CanardMicrosecond timestamp = 0;
        CanardFrame frame;
        frame.extended_can_id = rx_frame.can_id & CAN_EXT_ID_MASK;
        frame.payload_size = rx_frame.can_dlc;
        frame.payload = rx_frame.data;

        CanardRxTransfer transfer;
        CanardRxSubscription * sub;
        int32_t ret = canardRxAccept(&instance, timestamp, &frame, 0, &transfer, &sub);
        if(ret == 1) {
          //success

//          auto receiveTS = std::chrono::high_resolution_clock::now();
//          auto dt = receiveTS - lastReceiveTS;
//          lastReceiveTS = receiveTS;
          // process message
          publishReceivedMessage(&transfer);
          //then
          instance.memory_free(&instance, transfer.payload);

//            printf("did complete %u ------------------\n", count);
//            count = 0;

        } else if (ret == 0) {
//            printf("not complete %u\n", count);
//            count ++;
            // rejected because not subscribed or transfer not complete
        }else {
            //error
            printf("frame error %i\n", ret);
        }

      } else {
          printf("What?");
      }
//        std::cout << "dt: " << std::chrono::duration_cast<std::chrono::microseconds>(dt).count() << "µs\n";
    }
  }
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
    case ACTION_SERVO_GENERIC_READ_ID:{
      jeroboam_datatypes_actuators_servo_GenericReadResponse_0_1 response;
      int8_t res = jeroboam_datatypes_actuators_servo_GenericReadResponse_0_1_deserialize_(&response,
                                                                              (uint8_t *)transfer->payload,
                                                                              &transfer->payload_size);
      if(res == NUNAVUT_SUCCESS) {
        canBridge.get()->publishServoGenericReadResponse(response);
      } else {
        printf("ACTION_SERVO_GENERIC_READ_ID deserialize failed %i\r\n", res);
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