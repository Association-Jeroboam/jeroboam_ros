#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

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
#include "CanProtocol.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "jrb_msgs/msg/pid_state.hpp"
#include "jrb_msgs/msg/pump_status.hpp"
#include "jrb_msgs/msg/valve_status.hpp"
#include "jrb_msgs/msg/pid_config.hpp"
#include "jrb_msgs/msg/adaptative_pid_config.hpp"
#include "jrb_msgs/msg/motion_config.hpp"

class CanBridgeNode : public rclcpp::Node
{
  public:
    CanBridgeNode();

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

    void robot_twist_goal_cb(const geometry_msgs::msg::Twist::SharedPtr msg) const;
    void pumpLeftCB(const jrb_msgs::msg::PumpStatus::SharedPtr msg) const;
    void pumpRightCB(const jrb_msgs::msg::PumpStatus::SharedPtr msg) const;
    void valveLeftCB(const jrb_msgs::msg::ValveStatus::SharedPtr msg) const;
    void valveRightCB(const jrb_msgs::msg::ValveStatus::SharedPtr msg) const;
    void leftAdaptPidConfCB( const jrb_msgs::msg::AdaptativePIDConfig msg) const;
    
    void adaptPidConfCB( const jrb_msgs::msg::AdaptativePIDConfig msg) const;
    void motionConfigCB( const jrb_msgs::msg::MotionConfig msg) const;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       odom_pub;
    rclcpp::Publisher<jrb_msgs::msg::PIDState>::SharedPtr       left_pid_pub;
    rclcpp::Publisher<jrb_msgs::msg::PIDState>::SharedPtr       right_pid_pub;
    rclcpp::Publisher<jrb_msgs::msg::PumpStatus>::SharedPtr     left_pump_pub;
    rclcpp::Publisher<jrb_msgs::msg::PumpStatus>::SharedPtr     right_pump_pub;
    rclcpp::Publisher<jrb_msgs::msg::ValveStatus>::SharedPtr    left_valve_pub;
    rclcpp::Publisher<jrb_msgs::msg::ValveStatus>::SharedPtr    right_valve_pub;
  
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr  twist_sub;
    rclcpp::Subscription<jrb_msgs::msg::PumpStatus>::SharedPtr  left_pump_sub;
    rclcpp::Subscription<jrb_msgs::msg::PumpStatus>::SharedPtr  right_pump_sub;
    rclcpp::Subscription<jrb_msgs::msg::ValveStatus>::SharedPtr left_valve_sub;
    rclcpp::Subscription<jrb_msgs::msg::ValveStatus>::SharedPtr right_valve_sub;
    rclcpp::Subscription<jrb_msgs::msg::AdaptativePIDConfig>::SharedPtr left_adapt_pid_conf_sub;
    rclcpp::Subscription<jrb_msgs::msg::AdaptativePIDConfig>::SharedPtr right_adapt_pid_conf_sub;
    rclcpp::Subscription<jrb_msgs::msg::MotionConfig>::SharedPtr motion_config_sub;

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle;
};