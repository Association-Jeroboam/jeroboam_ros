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
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "jrb_msgs/msg/pid_state.hpp"
#include "jrb_msgs/msg/pid_config.hpp"
#include "jrb_msgs/msg/adaptative_pid_config.hpp"
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sysinfo.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "canard.h"
#include "Heartbeat_1_0.h"
#include "State2D_1_0.h"
#include "Twist2D_1_0.h"
#include "CanProtocol.hpp"
#include "PIDState_0_1.h"
#include "PumpStatus_0_1.h"
#include "ValveStatus_0_1.h"
#include "PIDConfig_0_1.h"
#include "AdaptativePIDConfig_0_1.h"
#include "MotionConfig_0_1.h"
#include "TurbineCmd_0_1.h"
#include "jrb_can_bridge/param_utils.hpp"
#include "jrb_msgs/msg/servo_angle.hpp"
#include "jrb_msgs/msg/servo_config.hpp"
#include "jrb_msgs/msg/servo_generic_command.hpp"
#include "jrb_msgs/msg/servo_generic_read.hpp"
#include "jrb_msgs/msg/servo_generic_read_response.hpp"
#include "jrb_msgs/msg/motion_speed_command.hpp"
#include "jrb_msgs/msg/odometry_ticks.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ServoAngle_0_1.h"
#include "ServoConfig_0_1.h"
#include "ServoID_0_1.h"
#include "GenericCommand_0_1.h"
#include "GenericRead_0_1.h"
#include "GenericReadResponse_0_1.h"
#include "SpeedCommand_0_1.h"
#include "OdometryTicks_0_1.h"
#include "CanBridgeTx.hpp"
#include "CanBridgeRx.hpp"

class CanBridge : public rclcpp::Node
{
  public:
    CanBridge();

    void init();
    void initSubs();
    void initPubs();

    void setAdaptPidParam(std::string side, std::string threshold, std::string param_name, double value);

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
  
    void publishRobotCurrentState(jeroboam_datatypes_sensors_odometry_State2D_1_0 * state);
    void publishLeftPIDState(jeroboam_datatypes_actuators_motion_PIDState_0_1* pid);
    void publishRightPIDState(jeroboam_datatypes_actuators_motion_PIDState_0_1* pid);
    void publishLinearPIDState(jeroboam_datatypes_actuators_motion_PIDState_0_1* pid);
    void publishAngularPIDState(jeroboam_datatypes_actuators_motion_PIDState_0_1* pid);
    void publishLeftPumpStatus(jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 * status);
    void publishRightPumpStatus(jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 * status);
    void publishLeftValveStatus(jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 * status);
    void publishRightValveStatus(jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 * status);
    void publishServoGenericReadResponse(jeroboam_datatypes_actuators_servo_GenericReadResponse_0_1 * response);
    void publishServoAngle(jeroboam_datatypes_actuators_servo_ServoAngle_0_1 * servoAngle);
    void publishOdometryTicks(jeroboam_datatypes_sensors_odometry_OdometryTicks_0_1 * odometryTicks);
    void sendMotionConfig();
    void sendAdaptPidConfig(std::string side);
    void publishEmergencyStop(bool * emergencyStop);
    bool init_done;

  private:
    static void send_can_msg(CanardPortID portID, CanardTransferID* transferID, void* buffer, size_t buf_size);

    static void send_can_request(CanardPortID portID, CanardNodeID destID, CanardTransferID* transferID, void* buffer, size_t buf_size);

    void robot_twist_goal_cb(const geometry_msgs::msg::Twist::SharedPtr msg);

    void initialpose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    void pumpLeftCB(const std_msgs::msg::Bool::SharedPtr msg);

    void pumpRightCB(const std_msgs::msg::Bool::SharedPtr msg);

    void valveLeftCB(const std_msgs::msg::Bool::SharedPtr msg);

    void valveRightCB(const std_msgs::msg::Bool::SharedPtr msg);

    void servoAngleCB (const jrb_msgs::msg::ServoAngle msg);

    void servoConfigCB (const jrb_msgs::msg::ServoConfig msg);

    void servoRebootCB (const std_msgs::msg::UInt8 msg);

    void servoGenericCommandCB (const jrb_msgs::msg::ServoGenericCommand msg);

    void servoGenericReadCB (const jrb_msgs::msg::ServoGenericRead msg);

    void motionSpeedCommandCB (const jrb_msgs::msg::MotionSpeedCommand msg);

    void turbineSpeedCB (const std_msgs::msg::UInt16 msg);

    void pwmCommandCB (const std_msgs::msg::Float32MultiArray msg);

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       odom_pub;
    rclcpp::Publisher<jrb_msgs::msg::PIDState>::SharedPtr       left_pid_pub;
    rclcpp::Publisher<jrb_msgs::msg::PIDState>::SharedPtr       right_pid_pub;
    rclcpp::Publisher<jrb_msgs::msg::PIDState>::SharedPtr       linear_pid_pub;
    rclcpp::Publisher<jrb_msgs::msg::PIDState>::SharedPtr       angular_pid_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr     left_pump_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr     right_pump_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    left_valve_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    right_valve_pub;
    rclcpp::Publisher<jrb_msgs::msg::ServoGenericReadResponse>::SharedPtr    servo_generic_read_response_pub;
    rclcpp::Publisher<jrb_msgs::msg::ServoAngle>::SharedPtr     servo_angle_pub;
    rclcpp::Publisher<jrb_msgs::msg::OdometryTicks>::SharedPtr  odometry_ticks_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           emergency_pub;
  
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr  twist_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr  left_pump_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr  right_pump_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_valve_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_valve_sub;
    rclcpp::Subscription<jrb_msgs::msg::AdaptativePIDConfig>::SharedPtr left_adapt_pid_conf_sub;
    rclcpp::Subscription<jrb_msgs::msg::AdaptativePIDConfig>::SharedPtr right_adapt_pid_conf_sub;
    rclcpp::Subscription<jrb_msgs::msg::ServoAngle>::SharedPtr   servo_angle_sub;
    rclcpp::Subscription<jrb_msgs::msg::ServoConfig>::SharedPtr  servo_config_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr      servo_reboot_sub;
    rclcpp::Subscription<jrb_msgs::msg::ServoGenericCommand>::SharedPtr            servo_generic_command_sub;
    rclcpp::Subscription<jrb_msgs::msg::ServoGenericRead>::SharedPtr               servo_generic_read_sub;
    rclcpp::Subscription<jrb_msgs::msg::MotionSpeedCommand>::SharedPtr             motion_speed_command_sub;

    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr       turbine_speed_sub;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pwm_command_sub;

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

    rclcpp::TimerBase::SharedPtr send_config_timer;

    jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1 leftAdaptConfig;
    jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1 rightAdaptConfig;
    jeroboam_datatypes_actuators_motion_MotionConfig_0_1 motionConfig;
    std::string robot_name;
    bool send_config_enabled;
};
