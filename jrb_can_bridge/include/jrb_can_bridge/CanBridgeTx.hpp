#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sysinfo.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "canard.h"

namespace TxThread {
void CanBridgeInitTxThread();
void* CanBridgeDeinitTxThread();
bool pushQueue(const CanardTransferMetadata* const metadata,
               const size_t                        payload_size,
               const void* const                   payload);
}