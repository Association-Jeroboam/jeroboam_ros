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

#include "LinuxCan.hpp"

using namespace std::chrono_literals;

int send_can_frame(struct can_frame * frame) {
    if (write(canIFace, frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        RCLCPP_ERROR(rclcpp::get_logger("CanBridge"), "send_can_frame Write ERROR");
        return 1;
    }

    std::this_thread::sleep_for(10ms);
    return 0;
}