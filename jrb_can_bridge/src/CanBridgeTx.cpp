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
        printf("\nexecution  mutex init failed\n");
    //   return 1;
    }
    if (pthread_mutex_init(&queue_lock, NULL) != 0)
    {  
      printf("\nqueue mutex init failed\n");
    }
    pthread_create(&txThread, NULL, &checkTxQueue, NULL);
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

bool TxThread::pushQueue(const CanardTransferMetadata* const metadata,
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
