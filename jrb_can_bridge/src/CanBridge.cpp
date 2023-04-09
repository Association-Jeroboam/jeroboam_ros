#include "CanBridge.hpp"


CanBridge::CanBridge()
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
    servo_generic_read_response_pub = this->create_publisher<jrb_msgs::msg::ServoGenericReadResponse>("servo_generic_read_response", 10);
    servo_angle_pub = this->create_publisher<jrb_msgs::msg::ServoAngle>("servo_angle", 10);
    odometry_ticks_pub = this->create_publisher<jrb_msgs::msg::OdometryTicks>("odometry_ticks", 10);

    rclcpp::QoS cmd_vel_qos = rclcpp::SensorDataQoS().keep_last(1);

    // Subscribers
    twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", cmd_vel_qos, std::bind(&CanBridge::robot_twist_goal_cb, this, std::placeholders::_1));

    initialpose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1, std::bind(&CanBridge::initialpose_cb, this, std::placeholders::_1));

    left_pump_sub = this->create_subscription<jrb_msgs::msg::PumpStatus>(
    "left_pump_status", 4, std::bind(&CanBridge::pumpLeftCB, this, std::placeholders::_1));
    right_pump_sub = this->create_subscription<jrb_msgs::msg::PumpStatus>(
    "right_pump_status", 4, std::bind(&CanBridge::pumpRightCB, this, std::placeholders::_1));
    left_valve_sub = this->create_subscription<jrb_msgs::msg::ValveStatus>(
    "left_valve_status", 4, std::bind(&CanBridge::valveLeftCB, this, std::placeholders::_1));
    right_valve_sub = this->create_subscription<jrb_msgs::msg::ValveStatus>(
    "right_valve_status", 4, std::bind(&CanBridge::valveRightCB, this, std::placeholders::_1));
    motion_config_sub = this->create_subscription<jrb_msgs::msg::MotionConfig>(
    "motion_config", 20, std::bind(&CanBridge::motionConfigCB, this, std::placeholders::_1));
    servo_angle_sub = this->create_subscription<jrb_msgs::msg::ServoAngle>(
    "servo_angle_target", 4, std::bind(&CanBridge::servoAngleCB, this, std::placeholders::_1));
    servo_config_sub = this->create_subscription<jrb_msgs::msg::ServoConfig>(
    "servo_config", 20, std::bind(&CanBridge::servoConfigCB, this, std::placeholders::_1));
    servo_reboot_sub = this->create_subscription<jrb_msgs::msg::ServoID>(
    "servo_reboot", 20, std::bind(&CanBridge::servoRebootCB, this, std::placeholders::_1));
    servo_generic_command_sub = this->create_subscription<jrb_msgs::msg::ServoGenericCommand>(
    "servo_generic_command", 20, std::bind(&CanBridge::servoGenericCommandCB, this, std::placeholders::_1));
    servo_generic_read_sub = this->create_subscription<jrb_msgs::msg::ServoGenericRead>(
    "servo_generic_read", 20, std::bind(&CanBridge::servoGenericReadCB, this, std::placeholders::_1));
    motion_speed_command_sub = this->create_subscription<jrb_msgs::msg::MotionSpeedCommand>(
    "speed_command", 1, std::bind(&CanBridge::motionSpeedCommandCB, this, std::placeholders::_1));
    
    param_callback_handle = this->add_on_set_parameters_callback(std::bind(&CanBridge::parametersCallback, this, std::placeholders::_1));

    // CAN messages
    leftAdaptConfig.ID = CAN_PROTOCOL_LEFT_SPEED_PID_ID;
    rightAdaptConfig.ID = CAN_PROTOCOL_RIGHT_SPEED_PID_ID;

    // Timers
    // send_config_timer = 
}

void CanBridge::init() {
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

void CanBridge::setAdaptPidParam(std::string side, std::string threshold, std::string param_name, double value) {
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

rcl_interfaces::msg::SetParametersResult CanBridge::parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
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

void CanBridge::publishRobotCurrentState(reg_udral_physics_kinematics_cartesian_State_0_1 * state)
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

void CanBridge::publishLeftPIDState(jeroboam_datatypes_actuators_motion_PIDState_0_1* pid){
    auto pid_msg = jrb_msgs::msg::PIDState();
    pid_msg.output = pid->output;
    pid_msg.setpoint = pid->setpoint;
    pid_msg.error = pid->_error;

    left_pid_pub->publish(pid_msg);
}

void CanBridge::publishRightPIDState(jeroboam_datatypes_actuators_motion_PIDState_0_1* pid) {
    auto pid_msg = jrb_msgs::msg::PIDState();
    pid_msg.output = pid->output;
    pid_msg.setpoint = pid->setpoint;
    pid_msg.error = pid->_error;

    right_pid_pub->publish(pid_msg);
}
void CanBridge::publishLeftPumpStatus(jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 * status){
    auto status_msg = jrb_msgs::msg::PumpStatus();
    status_msg.enabled = status->status.enabled.value;

    left_pump_pub->publish(status_msg);
}
void CanBridge::publishRightPumpStatus(jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 * status){
    auto status_msg = jrb_msgs::msg::PumpStatus();
    status_msg.enabled = status->status.enabled.value;

    right_pump_pub->publish(status_msg);
}
void CanBridge::publishLeftValveStatus(jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 * status){
    auto status_msg = jrb_msgs::msg::ValveStatus();
    status_msg.enabled = status->status.enabled.value;

    left_valve_pub->publish(status_msg);
}
void CanBridge::publishRightValveStatus(jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 * status){
    auto status_msg = jrb_msgs::msg::ValveStatus();
    status_msg.enabled = status->status.enabled.value;

    right_valve_pub->publish(status_msg);
}

void CanBridge::publishServoGenericReadResponse(jeroboam_datatypes_actuators_servo_GenericReadResponse_0_1 * response) {
    auto servo_response = jrb_msgs::msg::ServoGenericReadResponse();
    RCLCPP_ERROR_STREAM(this->get_logger(), "BROKEN DOES NOT WORK (yet.)");
    
    //if(response._tag_ == CAN_PROTOCOL_RESPONSE_SUCCESS) {
        //RCLCPP_ERROR_STREAM(this->get_logger(), "count %lu\r\n", response.data.count);
        //RCLCPP_ERROR_STREAM(this->get_logger(), "count %lu", response->data.count);
       //for(size_t i = 0; i < response->data.count; i++){
    //        servo_response.data[i] = response->data.elements[i];
        //}
        //servo_response.len = response->data.count;
      //  servo_response.len = 1;
    //} else if(response._tag_ == CAN_PROTOCOL_RESPONSE_ERROR){
    //    RCLCPP_ERROR_STREAM(this->get_logger(), "tag = %i\r\n", response._tag_);
    //    RCLCPP_ERROR_STREAM(this->get_logger(), "error = %u\r\n", response.err_code);
    //    servo_response.len = 0;
    //} else {
    //    RCLCPP_ERROR_STREAM(this->get_logger(), "unhandled tag %u\r\n", response._tag_);;
    //}
    
    //servo_generic_read_response_pub->publish(servo_response);
}

void CanBridge::publishServoAngle(jeroboam_datatypes_actuators_servo_ServoAngle_0_1 * servoAngle) {
    auto servo_angle_msg = jrb_msgs::msg::ServoAngle();
    servo_angle_msg.id = servoAngle->ID;
    servo_angle_msg.radian = servoAngle->angle.radian;
    servo_angle_pub->publish(servo_angle_msg);
}

void CanBridge::publishOdometryTicks(jeroboam_datatypes_sensors_odometry_OdometryTicks_0_1 * odometryTicks) {
    auto ticks = jrb_msgs::msg::OdometryTicks();
    ticks.left = odometryTicks->left;
    ticks.right = odometryTicks->right;
    odometry_ticks_pub->publish(ticks);
}


void CanBridge::send_can_msg(CanardPortID portID, CanardTransferID* transferID, void* buffer, size_t buf_size) {
    if (buffer == nullptr || buf_size == 0) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("CanBridge"), "Invalid buffer or buffer size\n");
        return;
    }

    CanardTransferMetadata metadata;
    metadata.priority = CanardPriorityNominal;
    metadata.transfer_kind = CanardTransferKindMessage;
    metadata.port_id = portID;
    metadata.remote_node_id = CANARD_NODE_ID_UNSET;
    metadata.transfer_id = *transferID;


    bool success = TxThread::pushQueue(&metadata, buf_size, buffer);
    if (!success ) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("CanBridge"), "Queue push failed\n");
    }
    (*transferID)++;
}

void CanBridge::send_can_request(CanardPortID portID, CanardNodeID destID, CanardTransferID* transferID, void* buffer, size_t buf_size) {
    CanardTransferMetadata metadata;
    metadata.priority = CanardPriorityNominal;
    metadata.transfer_kind = CanardTransferKindRequest;
    metadata.port_id = portID;
    metadata.remote_node_id = destID;
    metadata.transfer_id = *transferID;


    bool success = TxThread::pushQueue(&metadata, buf_size, buffer);
    if (!success ) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("CanBridge"), "Queue push failed\n");
    }
    (*transferID)++;
}

void CanBridge::robot_twist_goal_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (msg == nullptr) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Received a null message\n");
        return;
    }

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

    int8_t result = reg_udral_physics_kinematics_cartesian_Twist_0_1_serialize_(&twist, buffer, &buf_size);
    if (result != NUNAVUT_SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::robot_twist_goal_cb error: Failed serializing cartesian_Twist " << result);
    }

    send_can_msg(ROBOT_TWIST_GOAL_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::initialpose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
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

    int8_t res = reg_udral_physics_kinematics_cartesian_Pose_0_1_serialize_(&pose, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::initialpose_cb error: Failed serilializing cartesian_Pose " << res);
        return;
    }

    send_can_msg(ROBOT_SET_CURRENT_POSE_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::pumpLeftCB(const jrb_msgs::msg::PumpStatus::SharedPtr msg) {
    static CanardTransferID transfer_id = 0;
    jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 pumpStatus;
    pumpStatus.status.ID = CAN_PROTOCOL_PUMP_LEFT_ID;
    pumpStatus.status.enabled.value = msg->enabled;

    size_t buf_size = jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_serialize_(&pumpStatus, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::pumpLeftCB error: Failed serilializing pneumatics_PumpStatus " << res);
        return;
    }

    send_can_msg(ACTION_PUMP_SET_STATUS_ID, &transfer_id, buffer, buf_size);
    
}

void CanBridge::pumpRightCB(const jrb_msgs::msg::PumpStatus::SharedPtr msg) {
    static CanardTransferID transfer_id = 0;
    jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 pumpStatus;
    pumpStatus.status.ID = CAN_PROTOCOL_PUMP_RIGHT_ID;
    pumpStatus.status.enabled.value = msg->enabled;

    size_t buf_size = jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_serialize_(&pumpStatus, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::pumpRightCB error: Failed serilializing pneumatics_PumpStatus " << res);
        return;
    }

    send_can_msg(ACTION_PUMP_SET_STATUS_ID, &transfer_id, buffer, buf_size);
    
}

void CanBridge::valveLeftCB(const jrb_msgs::msg::ValveStatus::SharedPtr msg) {
    static CanardTransferID transfer_id = 0;

    jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 valveStatus;
    valveStatus.status.ID = CAN_PROTOCOL_VALVE_LEFT_ID;
    valveStatus.status.enabled.value = msg->enabled;

    size_t buf_size = jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_serialize_(&valveStatus, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::valveLeftCB error: Failed serilializing pneumatics_ValveStatus " << res);
        return;
    }

    send_can_msg(ACTION_VALVE_SET_STATUS_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::valveRightCB(const jrb_msgs::msg::ValveStatus::SharedPtr msg) {
    static CanardTransferID transfer_id = 0;

    jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 valveStatus;
    valveStatus.status.ID = CAN_PROTOCOL_VALVE_RIGHT_ID;
    valveStatus.status.enabled.value = msg->enabled;

    size_t buf_size = jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_serialize_(&valveStatus, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::valveRightCB error: Failed serilializing pneumatics_ValveStatus " << res);
        return;
    }


    send_can_msg(ACTION_VALVE_SET_STATUS_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::sendAdaptPidConfig(std::string side) {
    static CanardTransferID transfer_id = 0;
    size_t buf_size = jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res;
    if (side == "left") {
        res = jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_serialize_(&leftAdaptConfig, buffer, &buf_size);
    } else {
        res = jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_serialize_(&rightAdaptConfig, buffer, &buf_size);
    }

    if (res != NUNAVUT_SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::sendAdaptPidConfig error: Failed serilializing motion_AdaptativePIDConfig " << res);
        return;
    }

    send_can_msg(MOTION_SET_ADAPTATIVE_PID_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::motionConfigCB( const jrb_msgs::msg::MotionConfig msg) {
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

    int8_t res = jeroboam_datatypes_actuators_motion_MotionConfig_0_1_serialize_(&motionConfig, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::motionConfigCB error: Failed serilializing motion_MotionConfig " << res);
        return;
    }

    send_can_msg(MOTION_SET_MOTION_CONFIG_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::servoAngleCB(const jrb_msgs::msg::ServoAngle msg) {
    static CanardTransferID transfer_id = 0;

    jeroboam_datatypes_actuators_servo_ServoAngle_0_1 servoAngle;

    servoAngle.ID = msg.id;
    servoAngle.angle.radian = msg.radian;

    size_t buf_size = jeroboam_datatypes_actuators_servo_ServoAngle_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_servo_ServoAngle_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_servo_ServoAngle_0_1_serialize_(&servoAngle, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::servoAngleCB error: Failed serilializing servo_ServoAngle " << res);
        return;
    }

    send_can_msg(ACTION_SERVO_SET_ANGLE_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::servoConfigCB(const jrb_msgs::msg::ServoConfig msg) {
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

    int8_t res = jeroboam_datatypes_actuators_servo_ServoConfig_0_1_serialize_(&servoConfig, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::servoConfigCB error: Failed serilializing servo_ServoConfig " << res);
        return;
    
    }

    send_can_msg(ACTION_SERVO_SET_CONFIG_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::servoRebootCB(const jrb_msgs::msg::ServoID msg) {
    static CanardTransferID transfer_id = 0;

    jeroboam_datatypes_actuators_servo_ServoID_0_1 servoID;

    servoID.ID = msg.id;

    size_t buf_size = jeroboam_datatypes_actuators_servo_ServoID_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_servo_ServoID_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_servo_ServoID_0_1_serialize_(&servoID, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::servoRebootCB error: Failed serilializing servo_ServoID " << res);
        return;
    }

    send_can_msg(ACTION_SERVO_REBOOT_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::servoGenericCommandCB(const jrb_msgs::msg::ServoGenericCommand msg) {
    static CanardTransferID transfer_id = 0;

    jeroboam_datatypes_actuators_servo_GenericCommand_0_1 command;

    command.id = msg.id;
    command.addr = msg.addr;
    command.data.count = msg.len;
    for (int i=0; i<msg.len; ++i)
    {
      command.data.elements[i] = msg.data[i];
    }

    size_t buf_size = jeroboam_datatypes_actuators_servo_GenericCommand_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_servo_GenericCommand_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_servo_GenericCommand_0_1_serialize_(&command, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::servoGenericCommandCB error: Failed serilializing servo_GenericCommand " << res);
        return;
    }

    send_can_msg(ACTION_SERVO_GENERIC_COMMAND_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::servoGenericReadCB(const jrb_msgs::msg::ServoGenericRead msg) {
    static CanardTransferID transfer_id = 0;

    jeroboam_datatypes_actuators_servo_GenericRead_0_1 read;

    read.id = msg.id;
    read.addr = msg.addr;
    read.len = msg.len;

    size_t buf_size = jeroboam_datatypes_actuators_servo_GenericRead_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_servo_GenericRead_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_servo_GenericRead_0_1_serialize_(&read, buffer, &buf_size);

    if (res != NUNAVUT_SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::servoGenericReadCB error: Failed serilializing servo_GenericRead " << res);
        return;
    }

    send_can_request(ACTION_SERVO_GENERIC_READ_ID, CAN_PROTOCOL_ACTION_BOARD_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::motionSpeedCommandCB(const jrb_msgs::msg::MotionSpeedCommand msg) {
    static CanardTransferID transfer_id = 0;
    jeroboam_datatypes_actuators_motion_SpeedCommand_0_1 command;

    command.left.meter_per_second = msg.left;
    command.right.meter_per_second = msg.right;

    size_t buf_size = jeroboam_datatypes_actuators_motion_SpeedCommand_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_motion_SpeedCommand_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_motion_SpeedCommand_0_1_serialize_(&command, buffer, &buf_size);

    if (res != NUNAVUT_SUCCESS) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed serilializing speed command " << res);
        return;
    }

    send_can_msg(ROBOT_GOAL_SPEEDS_WHEELS_ID, &transfer_id, buffer, buf_size);
}
