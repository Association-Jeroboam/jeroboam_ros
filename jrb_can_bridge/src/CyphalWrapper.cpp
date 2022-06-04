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

constexpr int MAX_FRAME_SIZE = 8;
const uint32_t CAN_EXT_ID_MASK = (1 <<29) -1;
typedef void * (*THREADFUNCPTR)(void *);
using namespace std::chrono_literals;

void * canardSpecificAlloc(CanardInstance * instance, size_t amount) {
	(void) instance;
	return malloc(amount);
}

void canardSpecificFree(CanardInstance * instance, void * pointer) {
	(void)instance;
	if(pointer) free(pointer);
}

CyphalWrapper::CyphalWrapper(cyphalPublishMessageCB cb):
publishCB(cb) {

}

void CyphalWrapper::init() {
	char iface[] = "can0";

	initCAN(iface);
	initCanard();

	if (pthread_mutex_init(&tx_exec_lock, NULL) != 0)
	{
		printf("\nexecution  mutex init failed\n");
		return;
	}

	if (pthread_mutex_init(&queue_lock, NULL) != 0)
	{
		printf("\nqueue mutex init failed\n");
		return;
	}

	pthread_create(&txThread, NULL, (THREADFUNCPTR)&CyphalWrapper::checkTxQueue, this);
	pthread_create(&rxThread, NULL, (THREADFUNCPTR)&CyphalWrapper::checkRxMsg, this);
}

void CyphalWrapper::shutdown() {
	pthread_cancel(txThread);
	pthread_cancel(rxThread);
	void* retval;
	pthread_join(txThread, &retval);
	pthread_join(rxThread, &retval);
	(void)retval;
	close(canIFace);
}

void* CyphalWrapper::checkTxQueue(void*) {

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

void* CyphalWrapper::checkRxMsg(void*) {
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
					// process message
					if(publishCB){
						publishCB(&transfer);
					}

					//then
					instance.memory_free(&instance, transfer.payload);

				} else if (ret == 0) {
					// rejected because not subscribed or transfer not complete
				}else {
					//error
					printf("frame error %i\n", ret);
				}

			} else {
				printf("What?");
			}
		}
	}
}

bool CyphalWrapper::pushQueue(const CanardTransferMetadata* const metadata,
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

bool CyphalWrapper::subscribe(CanardTransferKind transfer_kind, CanardPortID port_id, size_t extent){

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

void CyphalWrapper::initCAN(char * iface) {

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

int CyphalWrapper::send_can_frame(struct can_frame * frame) {
	if (write(canIFace, frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
		perror("Write ERROR");
		return 1;
	}
	std::this_thread::sleep_for(10ms);
	return 0;
}

void CyphalWrapper::initCanard(void) {
	instance = canardInit(canardSpecificAlloc, canardSpecificFree);
	instance.node_id = CAN_PROTOCOL_EMBEDDED_COMPUTER_ID; // Embedded computer Node ID
	queue = canardTxInit(10000, MAX_FRAME_SIZE);
}
