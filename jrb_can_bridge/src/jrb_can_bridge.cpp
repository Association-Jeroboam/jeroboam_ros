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
#include "CanBridge.hpp"

using namespace std::chrono_literals;

constexpr int CAN_RX_MAX_SUBSCRIPTION = 32;
constexpr int MAX_FRAME_SIZE = 8;
const uint32_t CAN_EXT_ID_MASK = (1 <<29) -1;

bool check_parameter(char * iface, char * name, size_t n);
void initCAN(char * iface);
int send_can_frame(struct can_frame * frame);

void * canardSpecificAlloc(CanardInstance * instance, size_t amount);
void canardSpecificFree(CanardInstance * instance, void * pointer);
void initCanard(void);
void* checkTxQueue(void*);
void* checkRxMsg(void*);
bool pushQueue(const CanardTransferMetadata* const metadata,
               const size_t                        payload_size,
               const void* const                   payload);
bool subscribe(CanardTransferKind transfer_kind, CanardPortID port_id, size_t extent);

void publishReceivedMessage(CanardRxTransfer * transfer);
void createSubscriptions(void);

void print_usage(void) {
    printf("usage:\n\tchyphal_demo can_interface (can0, vcan0...)\n");
}

int canIFace;

pthread_mutex_t queue_lock;
pthread_cond_t  tx_exec_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t tx_exec_lock;

static CanardInstance instance;
CanardTxQueue  queue;
CanardRxSubscription subscriptions[CAN_RX_MAX_SUBSCRIPTION];
unsigned int subCnt;

pthread_t txThread, rxThread;

std::shared_ptr<CanBridge> canBridge;

int main(int argc, char * argv[])
{
  char iface[] = "can0";

  initCAN(iface);
  initCanard();
  createSubscriptions();


  if (pthread_mutex_init(&tx_exec_lock, NULL) != 0)
  {
      printf("\nexecution  mutex init failed\n");
      return 1;
  }

  if (pthread_mutex_init(&queue_lock, NULL) != 0)
  {
      printf("\nqueue mutex init failed\n");
      return 1;
  }

  rclcpp::init(argc, argv);
  

  canBridge = std::make_shared<CanBridge>();
  std::this_thread::sleep_for(100ms);
  pthread_create(&txThread, NULL, &checkTxQueue, NULL);
  pthread_create(&rxThread, NULL, &checkRxMsg, NULL);
  std::this_thread::sleep_for(100ms);
  canBridge.get()->init();
  rclcpp::spin(canBridge);
  rclcpp::shutdown();
  pthread_cancel(txThread);
  pthread_cancel(rxThread);
  void* retval;
  pthread_join(txThread, &retval);
  pthread_join(rxThread, &retval);
  (void)retval;
  close(canIFace);
  return 0;
}

void* checkTxQueue(void*) {

  while(true) {
    pthread_mutex_lock(&tx_exec_lock);
    pthread_cond_wait(&tx_exec_cond, &tx_exec_lock);


    const CanardTxQueueItem* item = canardTxPeek(&queue);

    while(item != NULL) {
      CanardTxQueueItem* extractedItem = canardTxPop(&queue, item);
      uint32_t           size          = item->frame.payload_size;

      do {
        struct can_frame frame;
        frame.can_id = item->frame.extended_can_id | 1 << 31;

        if (size >= MAX_FRAME_SIZE) {
            frame.can_dlc = MAX_FRAME_SIZE;
            size -= MAX_FRAME_SIZE;
        } else {
            frame.can_dlc= size;
            size      = 0;
        }
        memcpy(&frame.data, item->frame.payload, frame.can_dlc);

        send_can_frame(&frame);
      } while (size > 0);

      instance.memory_free(&instance, extractedItem);
      item = canardTxPeek(&queue);
    }

    pthread_mutex_unlock(&tx_exec_lock);

  }
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

bool pushQueue(const CanardTransferMetadata* const metadata,
               const size_t                        payload_size,
               const void* const                   payload) {
    bool success;
    pthread_mutex_lock(&queue_lock); // prevents other threads from pushing in the queue at the same time
    int32_t res = canardTxPush(&queue, &instance, 0, metadata, payload_size, payload);
    pthread_mutex_unlock(&queue_lock);
    pthread_mutex_lock(&tx_exec_lock);
    pthread_cond_signal(&tx_exec_cond);
    pthread_mutex_unlock(&tx_exec_lock);
    
    success = (0 <= res);
    return success;
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

int send_can_frame(struct can_frame * frame) {
    if (write(canIFace, frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write ERROR");
        return 1;
    }
    std::this_thread::sleep_for(10ms);
    return 0;
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

