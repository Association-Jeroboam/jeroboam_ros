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
#include <errno.h>

#include "LinuxCan.hpp"

using namespace std::chrono_literals;

int send_can_frame(struct can_frame * frame) {
    int32_t res = write(canIFace, frame, sizeof(struct can_frame)) ;
    if (res != sizeof(struct can_frame)) {
        RCLCPP_ERROR(rclcpp::get_logger("CanBridge"), "send_can_frame Write ERROR %ld != %d", sizeof(struct can_frame), errno);
        return 1;
    }

    return 0;
}