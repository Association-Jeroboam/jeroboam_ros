#pragma once
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sysinfo.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <memory>

#include "CanBridge.hpp"

extern int canIFace;
extern std::shared_ptr<CanBridge> canBridge;
constexpr int MAX_FRAME_SIZE = 8;

int send_can_frame(struct can_frame * frame);