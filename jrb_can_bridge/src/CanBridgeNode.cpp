#include "jrb_can_bridge/CanBridgeNode.hpp"
#include "jrb_can_bridge/param_utils.hpp"

class CanBridgeNode : public rclcpp::Node
{
  public:
    CanBridgeNode()
    : Node("can_bridge")
    {
      // Publishers
      odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("robot_current_state", 50);
      left_pid_pub = this->create_publisher<jrb_msgs::msg::PIDState>("left_pid_state", 10);
      right_pid_pub = this->create_publisher<jrb_msgs::msg::PIDState>("right_pid_state", 10);
      left_pump_pub = this->create_publisher<jrb_msgs::msg::PumpStatus>("left_pump_status", 10);
      right_pump_pub = this->create_publisher<jrb_msgs::msg::PumpStatus>("right_pump_status", 10);
      left_valve_pub = this->create_publisher<jrb_msgs::msg::ValveStatus>("left_valve_status", 10);
      right_valve_pub = this->create_publisher<jrb_msgs::msg::ValveStatus>("right_valve_status", 10);

      // Subscribers
      twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 50, std::bind(&CanBridgeNode::robot_twist_goal_cb, this, std::placeholders::_1));

      left_pump_sub = this->create_subscription<jrb_msgs::msg::PumpStatus>(
        "left_pump_status", 4, std::bind(&CanBridgeNode::pumpLeftCB, this, std::placeholders::_1));
      right_pump_sub = this->create_subscription<jrb_msgs::msg::PumpStatus>(
        "right_pump_status", 4, std::bind(&CanBridgeNode::pumpRightCB, this, std::placeholders::_1));
      left_valve_sub = this->create_subscription<jrb_msgs::msg::ValveStatus>(
        "left_valve_status", 4, std::bind(&CanBridgeNode::valveLeftCB, this, std::placeholders::_1));
      right_valve_sub = this->create_subscription<jrb_msgs::msg::ValveStatus>(
        "right_valve_status", 4, std::bind(&CanBridgeNode::valveRightCB, this, std::placeholders::_1));
      left_adapt_pid_conf_sub = this->create_subscription<jrb_msgs::msg::AdaptativePIDConfig>(
        "left_adapt_pid_conf", 4, std::bind(&CanBridgeNode::leftAdaptPidConfCB, this, std::placeholders::_1));
      right_adapt_pid_conf_sub = this->create_subscription<jrb_msgs::msg::AdaptativePIDConfig>(
        "right_adapt_pid_conf", 4, std::bind(&CanBridgeNode::adaptPidConfCB, this, std::placeholders::_1));
      motion_config_sub = this->create_subscription<jrb_msgs::msg::PumpStatus>(
        "motion_config", 4, std::bind(&CanBridgeNode::motionConfigCB, this, std::placeholders::_1));

      // Parameters
      ros2_utils::add_parameter((rclcpp::Node&)*this, std::string("pid/left/p"), rclcpp::ParameterValue(0.001), (ros2_utils::floating_point_range){ 0.0, 10.0, 0.0001 }, std::string("left pid motor proportional coef"), std::string(""), false);
      param_callback_handle = this->add_on_set_parameters_callback(std::bind(&CanBridgeNode::parametersCallback, this, std::placeholders::_1));
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
      for (const auto &parameter : parameters) {
        auto name = parameter.get_name();
        if (name == "pid/left/p") {
          auto value = parameter.as_double();
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


    void leftAdaptPidConfCB( const jrb_msgs::msg::AdaptativePIDConfig msg) const {
      static CanardTransferID transfer_id = 0;

      jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1 adaptConfig;
      adaptConfig.ID = msg.id;
      for(uint8_t conf_idx = 0; conf_idx < 3; conf_idx++) {
        for(uint8_t pid_idx = 0; pid_idx < 3; pid_idx++) {
          adaptConfig.configs[conf_idx].pid[pid_idx] = msg.configs[conf_idx].pid[pid_idx];
        }
        adaptConfig.configs[conf_idx].bias = msg.configs[conf_idx].bias;
        adaptConfig.thresholds[conf_idx] = msg.thresholds[conf_idx];
      }
      

      size_t buf_size = jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
      uint8_t buffer[jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

      jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_serialize_(&adaptConfig, buffer, &buf_size);

      send_can_msg(MOTION_SET_ADAPTATIVE_PID_ID, &transfer_id, buffer, buf_size);
    }
    
    void adaptPidConfCB( const jrb_msgs::msg::AdaptativePIDConfig msg) const {
      static CanardTransferID transfer_id = 0;

      jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1 adaptConfig;
      adaptConfig.ID = msg.id;
      for(uint8_t conf_idx = 0; conf_idx < 3; conf_idx++) {
        for(uint8_t pid_idx = 0; pid_idx < 3; pid_idx++) {
          adaptConfig.configs[conf_idx].pid[pid_idx] = msg.configs[conf_idx].pid[pid_idx];
        }
        adaptConfig.configs[conf_idx].bias = msg.configs[conf_idx].bias;
        adaptConfig.thresholds[conf_idx] = msg.thresholds[conf_idx];
      }
      

      size_t buf_size = jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
      uint8_t buffer[jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

      jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_serialize_(&adaptConfig, buffer, &buf_size);

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