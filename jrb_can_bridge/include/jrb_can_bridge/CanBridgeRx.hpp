#pragma once

#include "rclcpp/rclcpp.hpp"

namespace RxThread {
void CanBridgeInitRxThread();
void* CanBridgeDeinitRxThread();
}