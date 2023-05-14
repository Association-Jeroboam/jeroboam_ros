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
#include "CanBridgeTx.hpp"
#include "CanardInterface.hpp"
#include "LinuxCan.hpp"

pthread_t txThread;
pthread_cond_t  tx_exec_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t tx_exec_lock;
pthread_mutex_t queue_lock;

void* checkTxQueue(void*);


void TxThread::CanBridgeInitTxThread() {
    if (pthread_mutex_init(&tx_exec_lock, NULL) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CanBridge"), "TxThread::CanBridgeInitTxThread: execution  mutex init failed");
    //   return 1;
    }
    if (pthread_mutex_init(&queue_lock, NULL) != 0)
    {  
      RCLCPP_ERROR(rclcpp::get_logger("CanBridge"), "TxThread::CanBridgeInitTxThread: queue mutex init failed");
    }

    pthread_attr_t attr;
    pthread_attr_init(&attr);

    // Set thread priority to 98 (using SCHED_RR policy)
    struct sched_param param;
    param.sched_priority = 98;
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setschedparam(&attr, &param);

    // Prevent thread from inheriting the scheduler attributes of the main thread
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    int ret =  pthread_create(&txThread, &attr, &checkTxQueue, NULL);
    if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("CanBridge"), "Failed to create real-time TxThread");
        return;
    }

    pthread_attr_destroy(&attr);
}

void* TxThread::CanBridgeDeinitTxThread() {
    void* retval;
  
    pthread_cancel(txThread);
    pthread_join(txThread, &retval);
    return retval;
}

void* checkTxQueue(void*) {

  while(true) {
    pthread_mutex_lock(&tx_exec_lock);
    pthread_cond_wait(&tx_exec_cond, &tx_exec_lock);

    const CanardTxQueueItem* item = canardTxPeek(&queue);

    while(item != nullptr) {
      pthread_mutex_lock(&queue_lock);
      CanardTxQueueItem* extractedItem = canardTxPop(&queue, item);
      pthread_mutex_unlock(&queue_lock);
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

        if (item->frame.payload == nullptr) {
          RCLCPP_ERROR(rclcpp::get_logger("CanBridge"), "TxThread::checkTxQueue error: frame has an empty payload");
        } else {
          memcpy(&frame.data, item->frame.payload, frame.can_dlc);
          send_can_frame(&frame);
        }
      } while (size > 0);

      instance.memory_free(&instance, extractedItem);
      item = canardTxPeek(&queue);
    }
    pthread_mutex_unlock(&tx_exec_lock);

  }
}

bool TxThread::pushQueue(const CanardTransferMetadata* const metadata,
               const size_t                        payload_size,
               const void* const                   payload) {
    if (metadata == nullptr) {
      RCLCPP_ERROR(rclcpp::get_logger("CanBridge"), "TxThread::pushQueue error: empty metadata");
      return -1;
    }

    if (payload == nullptr) {
      RCLCPP_ERROR(rclcpp::get_logger("CanBridge"), "TxThread::pushQueue error: empty payload");
      return -1;
    }

    pthread_mutex_lock(&queue_lock); // prevents other threads from pushing in the queue at the same time
    int32_t res = canardTxPush(&queue, &instance, 0, metadata, payload_size, payload);
    pthread_mutex_unlock(&queue_lock);
    
    pthread_mutex_lock(&tx_exec_lock);
    pthread_cond_signal(&tx_exec_cond);
    pthread_mutex_unlock(&tx_exec_lock);

    
    bool success = (0 <= res);

    return success;
}
