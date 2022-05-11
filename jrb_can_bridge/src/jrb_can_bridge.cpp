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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sysinfo.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "canard.h"
#include "Heartbeat_1_0.h"
#include "cartesian/State_0_1.h"


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

class CanRxPublisher : public rclcpp::Node
{
  public:
    CanRxPublisher()
    : Node("Can_RX_Publisher")
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("robot_current_state", 50);
    }

  
    void publishRobotCurrentState(reg_udral_physics_kinematics_cartesian_State_0_1 * state)
    {
      auto state_msg = nav_msgs::msg::Odometry();
      state_msg.header.frame_id = "odom";
      state_msg.child_frame_id = "base_footprint";
      
      state_msg.pose.pose.position.x = state->pose.position.value.meter[0];
      state_msg.pose.pose.position.y = state->pose.position.value.meter[1];
      state_msg.pose.pose.position.z = state->pose.position.value.meter[2];

      state_msg.pose.pose.orientation.w = state->pose.orientation.wxyz[0],
      state_msg.pose.pose.orientation.x = state->pose.orientation.wxyz[1],
      state_msg.pose.pose.orientation.y = state->pose.orientation.wxyz[2],
      state_msg.pose.pose.orientation.z = state->pose.orientation.wxyz[3],


      state_msg.twist.twist.linear.x = state->twist.linear.meter_per_second[0];
      state_msg.twist.twist.linear.y = state->twist.linear.meter_per_second[1];
      state_msg.twist.twist.linear.z = state->twist.linear.meter_per_second[2];

      state_msg.twist.twist.angular.x = state->twist.angular.radian_per_second[0];
      state_msg.twist.twist.angular.y = state->twist.angular.radian_per_second[1];
      state_msg.twist.twist.angular.z = state->twist.angular.radian_per_second[2];

      publisher_->publish(state_msg);
    }
  private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

std::shared_ptr<CanRxPublisher> canRxPublisher;

int main(int argc, char * argv[])
{
    rclcpp::TimerBase::SharedPtr timer_;
  if(argc != 2) {
        print_usage();
        return 1;
  }
  char canName[] = "can";
  char vcanName[] = "vcan";
  bool hardware = check_parameter(argv[1], canName, 3);
  bool emulated = check_parameter(argv[1], vcanName, 4);


  if(!hardware && !emulated) {
      print_usage();
      return 2;
  }

  char * iface = argv[1];

  initCAN(iface);
  initCanard();
  CanardPortID portID = 0; // Robot Current State ID
  subscribe(CanardTransferKindMessage, portID, reg_udral_physics_kinematics_cartesian_State_0_1_EXTENT_BYTES_);
  subscribe(CanardTransferKindMessage, uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, uavcan_node_Heartbeat_1_0_EXTENT_BYTES_);

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
  canRxPublisher = std::make_shared<CanRxPublisher>();
  pthread_create(&txThread, NULL, &checkTxQueue, NULL);
  pthread_create(&rxThread, NULL, &checkRxMsg, NULL);
  rclcpp::spin(canRxPublisher);
  rclcpp::shutdown();
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
  while (true) {
    if(canIFace !=0) {
      struct can_frame rx_frame;
      //this is blocking
      int nbytes = read(canIFace, &rx_frame, sizeof(struct can_frame));

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

          // process message
          reg_udral_physics_kinematics_cartesian_State_0_1 state;
          // bool state_rcvd = false;
          if(transfer.metadata.port_id == 0) {

            // state_rcvd = true;
            reg_udral_physics_kinematics_cartesian_State_0_1_deserialize_(&state,
                                                                          (uint8_t *)transfer.payload,
                                                                          &transfer.payload_size);
            canRxPublisher.get()->publishRobotCurrentState(&state);
          }
          //then
          instance.memory_free(&instance, transfer.payload);

          // if(state_rcvd) {
          //   for (int i = 0; i < 3; ++i) {
          //     printf("%.4f ", state.pose.position.value.meter[i]);
          //   }
          //   printf("\n");
          //   for (int i = 0; i < 4; ++i) {
          //     printf("%.4f ", state.pose.orientation.wxyz[i]);
          //   }
          //   printf("\n");
          //   for (int i = 0; i < 3; ++i) {
          //     printf("%.4f ", state.twist.linear.meter_per_second[i]);
          //   }
          //   printf("\n");
          //   for (int i = 0; i < 3; ++i) {
          //     printf("%.4f ", state.twist.angular.radian_per_second[i]);
          //   }
          //   printf("\n");

          // }

        } else if (ret == 0) {
            // rejected because not subscribed or transfer not complete
        }else {
            //error
            printf("frame error %i\n", ret);
        }
      }
    } else {
      usleep(10);
    }
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
    return 0;
}

bool check_parameter(char * iface, char * name, size_t n) {
    return memcmp(iface, name, n) == 0;
}

void initCanard(void) {
    instance = canardInit(canardSpecificAlloc, canardSpecificFree);
    instance.node_id = 42; // Embedded computer Node ID
    queue = canardTxInit(100, MAX_FRAME_SIZE);
}

void * canardSpecificAlloc(CanardInstance * instance, size_t amount) {
    (void) instance;
    return malloc(amount);
}

void canardSpecificFree(CanardInstance * instance, void * pointer) {
    (void)instance;
    if(pointer) free(pointer);
}
