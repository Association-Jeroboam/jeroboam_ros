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

#include "rclcpp/rclcpp.hpp"
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
#include "ServoAngle_0_1.h"
#include "ServoConfig_0_1.h"

using namespace std::chrono_literals;

constexpr int CAN_RX_MAX_SUBSCRIPTION = 32;
constexpr int MAX_FRAME_SIZE = 8;
const uint32_t CAN_EXT_ID_MASK = (1 <<29) -1;

bool check_parameter(char * iface, char * name, size_t n);
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
void createSubscriptions(void);

void print_usage(void) {
    printf("usage:\n\tchyphal_demo can_interface (can0, vcan0...)\n");
}

int canIFace;

pthread_mutex_t queue_lock;
pthread_cond_t  tx_exec_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t tx_exec_lock;

static CanardInstance instance;
CanardTxQueue  queue;
CanardRxSubscription subscriptions[CAN_RX_MAX_SUBSCRIPTION];
unsigned int subCnt;

pthread_t txThread, rxThread;

class CanBridge : public rclcpp::Node
{
  public:
    CanBridge()
    : Node("can_bridge")
    {
      // Publishers
      odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 50);
      left_pid_pub = this->create_publisher<jrb_msgs::msg::PIDState>("left_pid_state", 10);
      right_pid_pub = this->create_publisher<jrb_msgs::msg::PIDState>("right_pid_state", 10);
      left_pump_pub = this->create_publisher<jrb_msgs::msg::PumpStatus>("left_pump_status", 10);
      right_pump_pub = this->create_publisher<jrb_msgs::msg::PumpStatus>("right_pump_status", 10);
      left_valve_pub = this->create_publisher<jrb_msgs::msg::ValveStatus>("left_valve_status", 10);
      right_valve_pub = this->create_publisher<jrb_msgs::msg::ValveStatus>("right_valve_status", 10);

      // Subscribers
      twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 50, std::bind(&CanBridge::robot_twist_goal_cb, this, std::placeholders::_1));

      initialpose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 50, std::bind(&CanBridge::initialpose_cb, this, std::placeholders::_1));

      left_pump_sub = this->create_subscription<jrb_msgs::msg::PumpStatus>(
        "left_pump_status", 4, std::bind(&CanBridge::pumpLeftCB, this, std::placeholders::_1));
      right_pump_sub = this->create_subscription<jrb_msgs::msg::PumpStatus>(
        "right_pump_status", 4, std::bind(&CanBridge::pumpRightCB, this, std::placeholders::_1));
      left_valve_sub = this->create_subscription<jrb_msgs::msg::ValveStatus>(
        "left_valve_status", 4, std::bind(&CanBridge::valveLeftCB, this, std::placeholders::_1));
      right_valve_sub = this->create_subscription<jrb_msgs::msg::ValveStatus>(
        "right_valve_status", 4, std::bind(&CanBridge::valveRightCB, this, std::placeholders::_1));
      motion_config_sub = this->create_subscription<jrb_msgs::msg::PumpStatus>(
        "motion_config", 4, std::bind(&CanBridge::motionConfigCB, this, std::placeholders::_1));
      servo_angle_sub = this->create_subscription<jrb_msgs::msg::ServoAngle>(
        "servo_angle_target", 4, std::bind(&CanBridge::servoAngleCB, this, std::placeholders::_1));
      servo_config_sub = this->create_subscription<jrb_msgs::msg::ServoConfig>(
<<<<<<< Updated upstream
        "servo_confif", 4, std::bind(&CanBridge::servoConfigCB, this, std::placeholders::_1));
=======
        "servo_config", 4, std::bind(&CanBridge::servoConfigCB, this, std::placeholders::_1));
>>>>>>> Stashed changes

      param_callback_handle = this->add_on_set_parameters_callback(std::bind(&CanBridge::parametersCallback, this, std::placeholders::_1));

      // CAN messages
      leftAdaptConfig.ID = CAN_PROTOCOL_LEFT_SPEED_PID_ID;
      rightAdaptConfig.ID = CAN_PROTOCOL_RIGHT_SPEED_PID_ID;

      // Timers
      // send_config_timer = 
    }

    void init() {
      // Parameters
      const auto sides = std::array<std::string, 2>({"left", "right"});
      const auto thresholds = std::array<std::string, 3>({"low", "medium", "high"});

      for (auto const& side : sides) {
        for (auto const& threshold : thresholds) {
          double value;

          ros2_utils::add_parameter((rclcpp::Node&)*this, std::string("pid/"+side+"/"+threshold+"/p"), rclcpp::ParameterValue(0.004), (ros2_utils::floating_point_range){0.0, 10.0, 0.0001}, std::string(side + " left pid motor proportional coef"), std::string(""), false);
          value = this->get_parameter("pid/"+side+"/"+threshold+"/p").as_double();
          setAdaptPidParam(side, threshold, "p", value);

          ros2_utils::add_parameter((rclcpp::Node&)*this, std::string("pid/"+side+"/"+threshold+"/i"), rclcpp::ParameterValue(0.0005), (ros2_utils::floating_point_range){0.0, 10.0, 0.0001}, std::string(side + " left pid motor integral coef"), std::string(""), false);
          value = this->get_parameter("pid/"+side+"/"+threshold+"/i").as_double();
          setAdaptPidParam(side, threshold, "i", value);

          ros2_utils::add_parameter((rclcpp::Node&)*this, std::string("pid/"+side+"/"+threshold+"/d"), rclcpp::ParameterValue(0.0), (ros2_utils::floating_point_range){0.0, 10.0, 0.0001}, std::string(side + " left pid motor derivative coef"), std::string(""), false);
          value = this->get_parameter("pid/"+side+"/"+threshold+"/d").as_double();
          setAdaptPidParam(side, threshold, "d", value);

          ros2_utils::add_parameter((rclcpp::Node&)*this, std::string("pid/"+side+"/"+threshold+"/bias"), rclcpp::ParameterValue(0.0), (ros2_utils::floating_point_range){0.0, 10.0, 0.0001}, std::string(side + " left pid motor bias"), std::string(""), false);
          value = this->get_parameter("pid/"+side+"/"+threshold+"/bias").as_double();
          setAdaptPidParam(side, threshold, "bias", value);

          ros2_utils::add_parameter((rclcpp::Node&)*this, std::string("pid/"+side+"/"+threshold+"/threshold"), rclcpp::ParameterValue(0.01), (ros2_utils::floating_point_range){0.0, 10.0, 0.0001}, std::string(side + " velocity threshlod"), std::string(""), false);
          value = this->get_parameter("pid/"+side+"/"+threshold+"/threshold").as_double();
          setAdaptPidParam(side, threshold, "threshold", value);
        }
      }
    }

    void setAdaptPidParam(std::string side, std::string threshold, std::string param_name, double value) {
        jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1* adaptConfig;
        RCLCPP_INFO(this->get_logger(), "%s %s %s %f", side.c_str(), threshold.c_str(), param_name.c_str(), value);

        if (side == "left") {
          adaptConfig = &leftAdaptConfig;
        } else {
          adaptConfig = &rightAdaptConfig;
        }

        size_t conf_idx = 0;
        if (threshold == "low") {
          conf_idx = 0; 
        } else if (threshold == "medium") {
          conf_idx = 1;
        } else {
          conf_idx = 2;
        }

        if (param_name == "bias") {
          adaptConfig->configs[conf_idx].bias = value;
        } else if (param_name == "threshold") {
          adaptConfig->thresholds[conf_idx] = value;
        } else {
          size_t pid_idx = 0;
          if (param_name == "p") {
            pid_idx = 0; 
          } else if (param_name == "i") {
            pid_idx = 1;
          } else {
            pid_idx = 2;
          }

          adaptConfig->configs[conf_idx].pid[pid_idx] = value;
        }

        sendAdaptPidConfig(side);
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
      for (const auto &parameter : parameters) {
        auto name = parameter.get_name();

        if (name.rfind("pid/", 0) == 0) {
          std::string side, threshold, param_name;
          std::stringstream s(name);
          std::string part;
          std::vector<std::string> parts;

          while (std::getline(s, part, '/')) {
            parts.push_back(part);
          }

          side = parts[1];
          threshold = parts[2];
          param_name = parts[3];
          auto value = parameter.as_double();

          setAdaptPidParam(side, threshold, param_name, value);
        }
      }

      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      result.reason = "success";

      RCLCPP_INFO(this->get_logger(), "Params updated");

      return result;
    }
  
    void publishRobotCurrentState(reg_udral_physics_kinematics_cartesian_State_0_1 * state)
    {
      auto state_msg = nav_msgs::msg::Odometry();
      state_msg.header.stamp = get_clock()->now();
      state_msg.header.frame_id = "odom";
      state_msg.child_frame_id = "base_footprint";
      
      state_msg.pose.pose.position.x = state->pose.position.value.meter[0];
      state_msg.pose.pose.position.y = state->pose.position.value.meter[1];
      state_msg.pose.pose.position.z = state->pose.position.value.meter[2];

      state_msg.pose.pose.orientation.w = state->pose.orientation.wxyz[0],
      state_msg.pose.pose.orientation.x = state->pose.orientation.wxyz[1],
      state_msg.pose.pose.orientation.y = state->pose.orientation.wxyz[2],
      state_msg.pose.pose.orientation.z = state->pose.orientation.wxyz[3],


      state_msg.twist.twist.linear.x = state->twist.linear.meter_per_second[0];
      state_msg.twist.twist.linear.y = state->twist.linear.meter_per_second[1];
      state_msg.twist.twist.linear.z = state->twist.linear.meter_per_second[2];

      state_msg.twist.twist.angular.x = state->twist.angular.radian_per_second[0];
      state_msg.twist.twist.angular.y = state->twist.angular.radian_per_second[1];
      state_msg.twist.twist.angular.z = state->twist.angular.radian_per_second[2];

      odom_pub->publish(state_msg);
    }

    void publishLeftPIDState(jeroboam_datatypes_actuators_motion_PIDState_0_1* pid){
      auto pid_msg = jrb_msgs::msg::PIDState();
      pid_msg.output = pid->output;
      pid_msg.setpoint = pid->setpoint;
      pid_msg.error = pid->_error;

      left_pid_pub->publish(pid_msg);
    }

    void publishRightPIDState(jeroboam_datatypes_actuators_motion_PIDState_0_1* pid) {
      auto pid_msg = jrb_msgs::msg::PIDState();
      pid_msg.output = pid->output;
      pid_msg.setpoint = pid->setpoint;
      pid_msg.error = pid->_error;

      right_pid_pub->publish(pid_msg);
    }
    void publishLeftPumpStatus(jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 * status){
      auto status_msg = jrb_msgs::msg::PumpStatus();
      status_msg.enabled = status->status.enabled.value;

      left_pump_pub->publish(status_msg);
    }
    void publishRightPumpStatus(jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 * status){
      auto status_msg = jrb_msgs::msg::PumpStatus();
      status_msg.enabled = status->status.enabled.value;

      right_pump_pub->publish(status_msg);
    }
    void publishLeftValveStatus(jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 * status){
      auto status_msg = jrb_msgs::msg::ValveStatus();
      status_msg.enabled = status->status.enabled.value;

      left_valve_pub->publish(status_msg);
    }
    void publishRightValveStatus(jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 * status){
      auto status_msg = jrb_msgs::msg::ValveStatus();
      status_msg.enabled = status->status.enabled.value;

      right_valve_pub->publish(status_msg);
    }
  private:
    static void send_can_msg(CanardPortID portID, CanardTransferID* transferID, void* buffer, size_t buf_size) {
       CanardTransferMetadata metadata;
        metadata.priority = CanardPriorityNominal;
        metadata.transfer_kind = CanardTransferKindMessage;
        metadata.port_id = portID;
        metadata.remote_node_id = CANARD_NODE_ID_UNSET;
        metadata.transfer_id = *transferID;


        bool success = pushQueue(&metadata, buf_size, buffer);
        if (!success ) {
            printf("Queue push failed\n");
        }
        (*transferID)++;
    }

    void robot_twist_goal_cb(const geometry_msgs::msg::Twist::SharedPtr msg) const {
        static CanardTransferID transfer_id = 0;
        reg_udral_physics_kinematics_cartesian_Twist_0_1 twist;
        twist.linear.meter_per_second[0] = msg->linear.x;
        twist.linear.meter_per_second[1] = msg->linear.y;
        twist.linear.meter_per_second[2] = msg->linear.z;

        twist.angular.radian_per_second[0] = msg->angular.x;
        twist.angular.radian_per_second[1] = msg->angular.y;
        twist.angular.radian_per_second[2] = msg->angular.z;

        size_t buf_size = reg_udral_physics_kinematics_cartesian_Twist_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
        uint8_t buffer[reg_udral_physics_kinematics_cartesian_Twist_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

        reg_udral_physics_kinematics_cartesian_Twist_0_1_serialize_(&twist, buffer, &buf_size);

        send_can_msg(ROBOT_TWIST_GOAL_ID, &transfer_id, buffer, buf_size);
    }

    void initialpose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) const {
        static CanardTransferID transfer_id = 0;
        reg_udral_physics_kinematics_cartesian_Pose_0_1 pose;

        pose.position.value.meter[0] = msg->pose.pose.position.x;
        pose.position.value.meter[1] = msg->pose.pose.position.y;
        pose.position.value.meter[2] = 0;

        pose.orientation.wxyz[0] = msg->pose.pose.orientation.w;
        pose.orientation.wxyz[1] = msg->pose.pose.orientation.x;
        pose.orientation.wxyz[2] = msg->pose.pose.orientation.y;
        pose.orientation.wxyz[3] = msg->pose.pose.orientation.z;

        size_t buf_size = reg_udral_physics_kinematics_cartesian_Pose_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
        uint8_t buffer[reg_udral_physics_kinematics_cartesian_Pose_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

        reg_udral_physics_kinematics_cartesian_Pose_0_1_serialize_(&pose, buffer, &buf_size);

        send_can_msg(ROBOT_SET_CURRENT_POSE_ID, &transfer_id, buffer, buf_size);
    }

    void pumpLeftCB(const jrb_msgs::msg::PumpStatus::SharedPtr msg) const {
        static CanardTransferID transfer_id = 0;
        jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 pumpStatus;
        pumpStatus.status.ID = CAN_PROTOCOL_PUMP_LEFT_ID;
        pumpStatus.status.enabled.value = msg->enabled;

        size_t buf_size = jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
        uint8_t buffer[jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

        jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_serialize_(&pumpStatus, buffer, &buf_size);

        send_can_msg(ACTION_PUMP_SET_STATUS_ID, &transfer_id, buffer, buf_size);
      
    }

    void pumpRightCB(const jrb_msgs::msg::PumpStatus::SharedPtr msg) const {
        static CanardTransferID transfer_id = 0;
        jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 pumpStatus;
        pumpStatus.status.ID = CAN_PROTOCOL_PUMP_RIGHT_ID;
        pumpStatus.status.enabled.value = msg->enabled;

        size_t buf_size = jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
        uint8_t buffer[jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

        jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_serialize_(&pumpStatus, buffer, &buf_size);

        send_can_msg(ACTION_PUMP_SET_STATUS_ID, &transfer_id, buffer, buf_size);
      
    }

    void valveLeftCB(const jrb_msgs::msg::ValveStatus::SharedPtr msg) const {
        static CanardTransferID transfer_id = 0;

        jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 valveStatus;
        valveStatus.status.ID = CAN_PROTOCOL_VALVE_LEFT_ID;
        valveStatus.status.enabled.value = msg->enabled;

        size_t buf_size = jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
        uint8_t buffer[jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

        jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_serialize_(&valveStatus, buffer, &buf_size);

        send_can_msg(ACTION_VALVE_SET_STATUS_ID, &transfer_id, buffer, buf_size);
    }

    void valveRightCB(const jrb_msgs::msg::ValveStatus::SharedPtr msg) const {
        static CanardTransferID transfer_id = 0;

        jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 valveStatus;
        valveStatus.status.ID = CAN_PROTOCOL_VALVE_RIGHT_ID;
        valveStatus.status.enabled.value = msg->enabled;

        size_t buf_size = jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
        uint8_t buffer[jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

        jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_serialize_(&valveStatus, buffer, &buf_size);

        send_can_msg(ACTION_VALVE_SET_STATUS_ID, &transfer_id, buffer, buf_size);
    }

    void sendAdaptPidConfig(std::string side) {
      static CanardTransferID transfer_id = 0;
      size_t buf_size = jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
      uint8_t buffer[jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

       if (side == "left") {
         jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_serialize_(&leftAdaptConfig, buffer, &buf_size);
       } else {
         jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_serialize_(&rightAdaptConfig, buffer, &buf_size);
       }

      send_can_msg(MOTION_SET_ADAPTATIVE_PID_ID, &transfer_id, buffer, buf_size);
    }

    void motionConfigCB( const jrb_msgs::msg::MotionConfig msg) const {
      static CanardTransferID transfer_id = 0;

      jeroboam_datatypes_actuators_motion_MotionConfig_0_1 motionConfig;
      
      motionConfig.wheel_base.meter = msg.wheel_base;
      motionConfig.left_wheel_radius.meter = msg.left_wheel_radius;
      motionConfig.right_wheel_radius.meter = msg.right_wheel_radius;
      motionConfig.maxLinearSpeed.meter_per_second = msg.max_linear_speed;
      motionConfig.maxAngularSpeed.radian_per_second = msg.max_angular_speed;
      motionConfig.maxLinearAccl.meter_per_second_per_second = msg.max_linear_accl;
      motionConfig.maxAngularAccl.radian_per_second_per_second = msg.max_angular_accl;


      size_t buf_size = jeroboam_datatypes_actuators_motion_MotionConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
      uint8_t buffer[jeroboam_datatypes_actuators_motion_MotionConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

      jeroboam_datatypes_actuators_motion_MotionConfig_0_1_serialize_(&motionConfig, buffer, &buf_size);

      send_can_msg(MOTION_SET_MOTION_CONFIG_ID, &transfer_id, buffer, buf_size);
    }

    void servoAngleCB (const jrb_msgs::msg::ServoAngle msg) const {
        static CanardTransferID transfer_id = 0;

        jeroboam_datatypes_actuators_servo_ServoAngle_0_1 servoAngle;

        servoAngle.ID = msg.id;
        servoAngle.angle.radian = msg.radian;

        size_t buf_size = jeroboam_datatypes_actuators_servo_ServoAngle_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
        uint8_t buffer[jeroboam_datatypes_actuators_servo_ServoAngle_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

        jeroboam_datatypes_actuators_servo_ServoAngle_0_1_serialize_(&servoAngle, buffer, &buf_size);

        send_can_msg(ACTION_SERVO_SET_ANGLE_ID, &transfer_id, buffer, buf_size);
    }

    void servoConfigCB (const jrb_msgs::msg::ServoConfig msg) const {
        static CanardTransferID transfer_id = 0;

        jeroboam_datatypes_actuators_servo_ServoConfig_0_1 servoConfig;

        servoConfig.ID = msg.id;
        servoConfig._torque_limit = msg.torque_limit;
        servoConfig.moving_speed = msg.moving_speed;
        servoConfig.pid.pid[0] = msg.pid.pid[0];
        servoConfig.pid.pid[1] = msg.pid.pid[1];
        servoConfig.pid.pid[2] = msg.pid.pid[2];

        size_t buf_size = jeroboam_datatypes_actuators_servo_ServoConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
        uint8_t buffer[jeroboam_datatypes_actuators_servo_ServoConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

        jeroboam_datatypes_actuators_servo_ServoConfig_0_1_serialize_(&servoConfig, buffer, &buf_size);

        send_can_msg(ACTION_SERVO_SET_CONFIG_ID, &transfer_id, buffer, buf_size);
    }


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

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

    rclcpp::TimerBase::SharedPtr send_config_timer;

    jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1 leftAdaptConfig;
    jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1 rightAdaptConfig;
};

std::shared_ptr<CanBridge> canBridge;

int main(int argc, char * argv[])
{
  char iface[] = "can0";

  initCAN(iface);
  initCanard();
  createSubscriptions();


  if (pthread_mutex_init(&tx_exec_lock, NULL) != 0)
  {
      printf("\nexecution  mutex init failed\n");
      return 1;
  }

  if (pthread_mutex_init(&queue_lock, NULL) != 0)
  {
      printf("\nqueue mutex init failed\n");
      return 1;
  }

  rclcpp::init(argc, argv);
  

  canBridge = std::make_shared<CanBridge>();
  std::this_thread::sleep_for(100ms);
  pthread_create(&txThread, NULL, &checkTxQueue, NULL);
  pthread_create(&rxThread, NULL, &checkRxMsg, NULL);
  std::this_thread::sleep_for(100ms);
  canBridge.get()->init();
  rclcpp::spin(canBridge);
  rclcpp::shutdown();
  pthread_cancel(txThread);
  pthread_cancel(rxThread);
  void* retval;
  pthread_join(txThread, &retval);
  pthread_join(rxThread, &retval);
  (void)retval;
  close(canIFace);
  return 0;
}

void* checkTxQueue(void*) {

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

void* checkRxMsg(void*) {
//  auto lastReceiveTS = std::chrono::high_resolution_clock::now();
//  uint32_t count = 0;
  while (true) {
    if(canIFace !=0) {
      struct can_frame rx_frame;
      //this is blocking
      int nbytes = read(canIFace, &rx_frame, sizeof(struct can_frame));
//        auto receiveTS = std::chrono::high_resolution_clock::now();
//        auto dt = receiveTS - lastReceiveTS;
//        lastReceiveTS = receiveTS;

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
          //success

//          auto receiveTS = std::chrono::high_resolution_clock::now();
//          auto dt = receiveTS - lastReceiveTS;
//          lastReceiveTS = receiveTS;
          // process message
          publishReceivedMessage(&transfer);
          //then
          instance.memory_free(&instance, transfer.payload);

//            printf("did complete %u ------------------\n", count);
//            count = 0;

        } else if (ret == 0) {
//            printf("not complete %u\n", count);
//            count ++;
            // rejected because not subscribed or transfer not complete
        }else {
            //error
            printf("frame error %i\n", ret);
        }

      } else {
          printf("What?");
      }
//        std::cout << "dt: " << std::chrono::duration_cast<std::chrono::microseconds>(dt).count() << "µs\n";
    }
  }
}

void createSubscriptions(void) {
  subscribe(CanardTransferKindMessage,
            ROBOT_CURRENT_STATE_ID,
            reg_udral_physics_kinematics_cartesian_State_0_1_EXTENT_BYTES_);
  subscribe(CanardTransferKindMessage,
            uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
            uavcan_node_Heartbeat_1_0_EXTENT_BYTES_);
  subscribe(CanardTransferKindMessage,
            MOTION_PID_STATE_ID,
            jeroboam_datatypes_actuators_motion_PIDState_0_1_EXTENT_BYTES_);
  subscribe(CanardTransferKindMessage,
            ACTION_PUMP_STATUS_ID,
            jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_EXTENT_BYTES_);
  subscribe(CanardTransferKindMessage,
            ACTION_VALVE_STATUS_ID,
            jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_EXTENT_BYTES_); 
}

void publishReceivedMessage(CanardRxTransfer * transfer) {
  static uint32_t last_transfer_id =                    0;
  static uint64_t frameCount = 0;
  static uint64_t frameErrorCount = 0;
  switch (transfer->metadata.port_id)
  {
    case ROBOT_CURRENT_STATE_ID:{
        frameCount++;
      if((last_transfer_id +1) % 32 != transfer->metadata.transfer_id) {
          frameErrorCount++;
          printf("Transfer lost! %u %u. rate %.4f\n", last_transfer_id, transfer->metadata.transfer_id, (float)frameErrorCount/(float)frameCount);
      }
      last_transfer_id = transfer->metadata.transfer_id;
      reg_udral_physics_kinematics_cartesian_State_0_1 state;
      reg_udral_physics_kinematics_cartesian_State_0_1_deserialize_(&state,
                                                                  (uint8_t *)transfer->payload,
                                                                  &transfer->payload_size);
      canBridge.get()->publishRobotCurrentState(&state);
      break;
    }
    case MOTION_PID_STATE_ID:{
      jeroboam_datatypes_actuators_motion_PIDState_0_1 pidState;
      jeroboam_datatypes_actuators_motion_PIDState_0_1_deserialize_(&pidState,
                                                                    (uint8_t *)transfer->payload,
                                                                    &transfer->payload_size);
      if(pidState.ID == 0) {
        canBridge.get()->publishLeftPIDState(&pidState);
      } else {
        canBridge.get()->publishRightPIDState(&pidState);
      }
      break;
    }
    case ACTION_PUMP_STATUS_ID:{
      jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 status;
      jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_deserialize_(&status,
                                                                    (uint8_t *)transfer->payload,
                                                                    &transfer->payload_size);
      if(status.status.ID == 0) {
        canBridge.get()->publishLeftPumpStatus(&status);
      } else {
        canBridge.get()->publishRightPumpStatus(&status);
      }
      break;
    }
    case ACTION_VALVE_STATUS_ID:{
      jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 status;
      jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_deserialize_(&status,
                                                                    (uint8_t *)transfer->payload,
                                                                    &transfer->payload_size);
      if(status.status.ID == 0) {
        canBridge.get()->publishLeftValveStatus(&status);
      } else {
        canBridge.get()->publishRightValveStatus(&status);
      }
      break;
    }
    case uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_:
        // Heartbeat
        break;
    default:
      printf("Accepted a non handled CAN message(ID %u)! Please fix me!\n", transfer->metadata.port_id);
      break;
  }
}

bool pushQueue(const CanardTransferMetadata* const metadata,
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

bool subscribe(CanardTransferKind transfer_kind, CanardPortID port_id, size_t extent){

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

void initCAN(char * iface) {

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

int send_can_frame(struct can_frame * frame) {
    if (write(canIFace, frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write ERROR");
        return 1;
    }
    std::this_thread::sleep_for(10ms);
    return 0;
}

bool check_parameter(char * iface, char * name, size_t n) {
    return memcmp(iface, name, n) == 0;
}

void initCanard(void) {
    instance = canardInit(canardSpecificAlloc, canardSpecificFree);
    instance.node_id = CAN_PROTOCOL_EMBEDDED_COMPUTER_ID; // Embedded computer Node ID
    queue = canardTxInit(10000, MAX_FRAME_SIZE);
}

void * canardSpecificAlloc(CanardInstance * instance, size_t amount) {
    (void) instance;
    return malloc(amount);
}

void canardSpecificFree(CanardInstance * instance, void * pointer) {
    (void)instance;
    if(pointer) free(pointer);
}

