#ifndef __CYPHALWRAPPER_H__
#define __CYPHALWRAPPER_H__

#include  <inttypes.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sysinfo.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "canard.h"

typedef void *cyphalPublishMessageCB(CanardRxTransfer * transfer);

class CyphalWrapper {

public:
	CyphalWrapper(cyphalPublishMessageCB * cb);
	void init();
	void shutdown();
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

private:
	int canIFace;

	pthread_mutex_t queue_lock;
	pthread_cond_t  tx_exec_cond = PTHREAD_COND_INITIALIZER;
	pthread_mutex_t tx_exec_lock;

	CanardInstance instance;
	CanardTxQueue  queue;
	CanardRxSubscription subscriptions[CAN_RX_MAX_SUBSCRIPTION];
	unsigned int subCnt;
	cyphalPublishMessageCB publishCB;

	pthread_t txThread, rxThread;
};

#endif /* __CYPHALWRAPPER_H__ */
