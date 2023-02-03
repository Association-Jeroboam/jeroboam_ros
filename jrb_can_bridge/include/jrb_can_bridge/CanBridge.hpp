#pragma once
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

// #include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "jrb_msgs/msg/pid_state.hpp"
#include "jrb_msgs/msg/pump_status.hpp"
#include "jrb_msgs/msg/valve_status.hpp"
#include "jrb_msgs/msg/pid_config.hpp"
#include "jrb_msgs/msg/adaptative_pid_config.hpp"
#include "jrb_msgs/msg/motion_config.hpp"
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sysinfo.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "canard.h"
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
#include "jrb_msgs/msg/servo_id.hpp"
#include "ServoAngle_0_1.h"
#include "ServoConfig_0_1.h"
#include "ServoID_0_1.h"
#include "CanBridgeTx.hpp"
#include "CanBridgeRx.hpp"

class CanBridge : public rclcpp::Node
{
  public:
    CanBridge();

    void init();

    void setAdaptPidParam(std::string side, std::string threshold, std::string param_name, double value);

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
  
    void publishRobotCurrentState(reg_udral_physics_kinematics_cartesian_State_0_1 * state);
    void publishLeftPIDState(jeroboam_datatypes_actuators_motion_PIDState_0_1* pid);
    void publishRightPIDState(jeroboam_datatypes_actuators_motion_PIDState_0_1* pid);
    void publishLeftPumpStatus(jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 * status);
    void publishRightPumpStatus(jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 * status);
    void publishLeftValveStatus(jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 * status);
    void publishRightValveStatus(jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 * status);
  private:
    static void send_can_msg(CanardPortID portID, CanardTransferID* transferID, void* buffer, size_t buf_size);

    void robot_twist_goal_cb(const geometry_msgs::msg::Twist::SharedPtr msg);

    void initialpose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    void pumpLeftCB(const jrb_msgs::msg::PumpStatus::SharedPtr msg);

    void pumpRightCB(const jrb_msgs::msg::PumpStatus::SharedPtr msg);

    void valveLeftCB(const jrb_msgs::msg::ValveStatus::SharedPtr msg);

    void valveRightCB(const jrb_msgs::msg::ValveStatus::SharedPtr msg);

    void sendAdaptPidConfig(std::string side);

    void motionConfigCB( const jrb_msgs::msg::MotionConfig msg);

    void servoAngleCB (const jrb_msgs::msg::ServoAngle msg);

    void servoConfigCB (const jrb_msgs::msg::ServoConfig msg);

    void servoRebootCB (const jrb_msgs::msg::ServoID msg);


      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       odom_pub;
      rclcpp::Publisher<jrb_msgs::msg::PIDState>::SharedPtr       left_pid_pub;
      rclcpp::Publisher<jrb_msgs::msg::PIDState>::SharedPtr       right_pid_pub;
      rclcpp::Publisher<jrb_msgs::msg::PumpStatus>::SharedPtr     left_pump_pub;
      rclcpp::Publisher<jrb_msgs::msg::PumpStatus>::SharedPtr     right_pump_pub;
      rclcpp::Publisher<jrb_msgs::msg::ValveStatus>::SharedPtr    left_valve_pub;
      rclcpp::Publisher<jrb_msgs::msg::ValveStatus>::SharedPtr    right_valve_pub;
    
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr  twist_sub;
      rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub;
      rclcpp::Subscription<jrb_msgs::msg::PumpStatus>::SharedPtr  left_pump_sub;
      rclcpp::Subscription<jrb_msgs::msg::PumpStatus>::SharedPtr  right_pump_sub;
      rclcpp::Subscription<jrb_msgs::msg::ValveStatus>::SharedPtr left_valve_sub;
      rclcpp::Subscription<jrb_msgs::msg::ValveStatus>::SharedPtr right_valve_sub;
      rclcpp::Subscription<jrb_msgs::msg::AdaptativePIDConfig>::SharedPtr left_adapt_pid_conf_sub;
      rclcpp::Subscription<jrb_msgs::msg::AdaptativePIDConfig>::SharedPtr right_adapt_pid_conf_sub;
      rclcpp::Subscription<jrb_msgs::msg::MotionConfig>::SharedPtr motion_config_sub;
      rclcpp::Subscription<jrb_msgs::msg::ServoAngle>::SharedPtr   servo_angle_sub;
      rclcpp::Subscription<jrb_msgs::msg::ServoConfig>::SharedPtr  servo_config_sub;
      rclcpp::Subscription<jrb_msgs::msg::ServoID>::SharedPtr      servo_reboot_sub;

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

    rclcpp::TimerBase::SharedPtr send_config_timer;

    jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1 leftAdaptConfig;
    jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1 rightAdaptConfig;
};
