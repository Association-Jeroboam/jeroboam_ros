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
#include "CanBridgeTx.hpp"
#include "CanBridgeRx.hpp"
#include "LinuxCan.hpp"
#include "CanBridge.hpp"


using namespace std::chrono_literals;

constexpr int CAN_RX_MAX_SUBSCRIPTION = 32;

bool check_parameter(char * iface, char * name, size_t n);
void initCAN(char * iface);
int send_can_frame(struct can_frame * frame);

void * canardSpecificAlloc(CanardInstance * instance, size_t amount);
void canardSpecificFree(CanardInstance * instance, void * pointer);
void initCanard(void);
bool subscribe(CanardTransferKind transfer_kind, CanardPortID port_id, size_t extent);

void createSubscriptions(void);

void print_usage(void) {
    printf("usage:\n\tchyphal_demo can_interface (can0, vcan0...)\n");
}

int canIFace;


CanardInstance instance;
CanardTxQueue  queue;
CanardRxSubscription subscriptions[CAN_RX_MAX_SUBSCRIPTION];
unsigned int subCnt;

int main(int argc, char * argv[])
{
  char iface[] = "can0";

  initCAN(iface);
  initCanard();
  createSubscriptions();

  rclcpp::init(argc, argv);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  canBridge = std::make_shared<CanBridge>();
  executor.add_node(canBridge);
  
  // Threads init
  std::this_thread::sleep_for(100ms);
  TxThread::CanBridgeInitTxThread();
  RxThread::CanBridgeInitRxThread();
  std::this_thread::sleep_for(100ms);

  canBridge.get()->init();
  executor.spin();

  rclcpp::shutdown();

  // Threads join
  TxThread::CanBridgeDeinitTxThread();
  RxThread::CanBridgeDeinitRxThread();
  
  close(canIFace);

  return 0;
}


void createSubscriptions(void) {
  subscribe(CanardTransferKindMessage,
            ROBOT_CURRENT_STATE_ID,
            reg_udral_physics_kinematics_cartesian_State_0_1_EXTENT_BYTES_);
  subscribe(CanardTransferKindMessage,
            uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
            uavcan_node_Heartbeat_1_0_EXTENT_BYTES_);
  subscribe(CanardTransferKindMessage,
            MOTION_PID_STATE_ID,
            jeroboam_datatypes_actuators_motion_PIDState_0_1_EXTENT_BYTES_);
  subscribe(CanardTransferKindMessage,
            ACTION_PUMP_STATUS_ID,
            jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_EXTENT_BYTES_);
  subscribe(CanardTransferKindMessage,
            ACTION_VALVE_STATUS_ID,
            jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_EXTENT_BYTES_); 
  subscribe(CanardTransferKindResponse,
            ACTION_SERVO_GENERIC_READ_ID,
            jeroboam_datatypes_actuators_servo_GenericReadResponse_0_1_EXTENT_BYTES_);
  subscribe(CanardTransferKindMessage,
            ACTION_SERVO_CURRENT_ANGLE_ID,
            jeroboam_datatypes_actuators_servo_ServoAngle_0_1_EXTENT_BYTES_);
  subscribe(CanardTransferKindMessage,
            MOTION_ODOM_TICKS_ID,
            jeroboam_datatypes_sensors_odometry_OdometryTicks_0_1_EXTENT_BYTES_);
  subscribe(CanardTransferKindResponse,
            ACTION_SERVO_GENERIC_COMMAND_ID,
            uavcan_primitive_scalar_Integer8_1_0_EXTENT_BYTES_);    
}

bool subscribe(CanardTransferKind transfer_kind, CanardPortID port_id, size_t extent){

    if(subCnt >= CAN_RX_MAX_SUBSCRIPTION) return false;

    int32_t subRet = canardRxSubscribe(&instance,
                                       transfer_kind,
                                       port_id,
                                       extent,
                                       CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                       &subscriptions[subCnt]);
    if(subRet == 1) {
    // subscription created

    } else if(subRet == 0) {
    // Subscription already existing
    //TODO
    }

    bool res = subRet >= 0;
    if(res) {
        subCnt++;
    }
    return res;
}

void initCAN(char * iface) {
    // stolen from the basic tutorial
    struct sockaddr_can addr;
    struct ifreq ifr;


    if ((canIFace = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket");
        return;
    }

    strcpy(ifr.ifr_name, iface );
    ioctl(canIFace, SIOCGIFINDEX, &ifr);

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(canIFace, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        return;
    }
}

bool check_parameter(char * iface, char * name, size_t n) {
    return memcmp(iface, name, n) == 0;
}

void initCanard(void) {
    instance = canardInit(canardSpecificAlloc, canardSpecificFree);
    instance.node_id = CAN_PROTOCOL_EMBEDDED_COMPUTER_ID; // Embedded computer Node ID
    queue = canardTxInit(10000, MAX_FRAME_SIZE);
}

void * canardSpecificAlloc(CanardInstance * instance, size_t amount) {
    (void) instance;
    return malloc(amount);
}

void canardSpecificFree(CanardInstance * instance, void * pointer) {
    (void)instance;
    if(pointer) free(pointer);
}
