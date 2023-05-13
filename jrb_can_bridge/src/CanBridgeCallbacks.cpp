#include "CanBridge.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

void CanBridge::initSubs() {
    rclcpp::QoS cmd_vel_qos = rclcpp::SensorDataQoS().keep_last(1);
    static const rclcpp::QoS qos_profile = rclcpp::QoS(10)
            .history(RMW_QOS_POLICY_HISTORY_KEEP_ALL)
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    // Subscribers
    twist_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", cmd_vel_qos, std::bind(&CanBridge::robot_twist_goal_cb, this, std::placeholders::_1));

    initialpose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 1, std::bind(&CanBridge::initialpose_cb, this, std::placeholders::_1));

    servo_angle_sub = this->create_subscription<jrb_msgs::msg::ServoAngle>(
        "/hardware/servo/target_angle", qos_profile, std::bind(&CanBridge::servoAngleCB, this, std::placeholders::_1));
    servo_config_sub = this->create_subscription<jrb_msgs::msg::ServoConfig>(
        "/hardware/servo/config", qos_profile, std::bind(&CanBridge::servoConfigCB, this, std::placeholders::_1));
    servo_reboot_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "/hardware/servo/reboot", qos_profile, std::bind(&CanBridge::servoRebootCB, this, std::placeholders::_1));
    servo_generic_command_sub = this->create_subscription<jrb_msgs::msg::ServoGenericCommand>(
        "/hardware/servo/generic_command", qos_profile, std::bind(&CanBridge::servoGenericCommandCB, this, std::placeholders::_1));
    servo_generic_read_sub = this->create_subscription<jrb_msgs::msg::ServoGenericRead>(
        "/hardware/servo/generic_read", qos_profile, std::bind(&CanBridge::servoGenericReadCB, this, std::placeholders::_1));

    motion_speed_command_sub = this->create_subscription<jrb_msgs::msg::MotionSpeedCommand>(
        "/hardware/base/speed_command", qos_profile, std::bind(&CanBridge::motionSpeedCommandCB, this, std::placeholders::_1));

    pwm_command_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/hardware/base/pwm_command", qos_profile, std::bind(&CanBridge::pwmCommandCB, this, std::placeholders::_1));

    if (robot_name == "robotrouge")
    {
        left_pump_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/hardware/pump/left/set_status", 4, std::bind(&CanBridge::pumpLeftCB, this, std::placeholders::_1));
        right_pump_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/hardware/pump/right/set_status", 4, std::bind(&CanBridge::pumpRightCB, this, std::placeholders::_1));

        left_valve_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/hardware/valve/left/set_status", 4, std::bind(&CanBridge::valveLeftCB, this, std::placeholders::_1));
        right_valve_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/hardware/valve/right/set_status", 4, std::bind(&CanBridge::valveRightCB, this, std::placeholders::_1));
    } 
    else if (robot_name == "robotbleu")
    {
        turbine_speed_sub = this->create_subscription<std_msgs::msg::UInt16>(
            "/hardware/turbine/speed", 1, std::bind(&CanBridge::turbineSpeedCB, this, std::placeholders::_1));
    }
}

void CanBridge::robot_twist_goal_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (msg == nullptr)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Received a null message\n");
        return;
    }

    static CanardTransferID transfer_id = 0;
    jeroboam_datatypes_sensors_odometry_Twist2D_1_0 twist;

    twist.linear.meter_per_second = msg->linear.x;

    twist.angular.radian_per_second = msg->angular.z;

    size_t buf_size = jeroboam_datatypes_sensors_odometry_Twist2D_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_sensors_odometry_Twist2D_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t result = jeroboam_datatypes_sensors_odometry_Twist2D_1_0_serialize_(&twist, buffer, &buf_size);
    if (result != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::robot_twist_goal_cb error: Failed serializing cartesian_Twist " << result);
    }

    send_can_msg(ROBOT_TWIST_GOAL_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::initialpose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    static CanardTransferID transfer_id = 0;
    jeroboam_datatypes_sensors_odometry_Pose2D_1_0 pose;

    pose.x.meter = msg->pose.pose.position.x;
    pose.y.meter = msg->pose.pose.position.y;

    // This is supposed to work... but has to be tested
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(msg->pose.pose.orientation, tf2_quat);
    tf2::Matrix3x3 m(tf2_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    pose.theta.radian = yaw;

    size_t buf_size = jeroboam_datatypes_sensors_odometry_Pose2D_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_sensors_odometry_Pose2D_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_sensors_odometry_Pose2D_1_0_serialize_(&pose, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::initialpose_cb error: Failed serilializing cartesian_Pose " << res);
        return;
    }

    send_can_msg(ROBOT_SET_CURRENT_POSE_ID, &transfer_id, buffer, buf_size);
    // Double it in case we restarted the node and we already sent a current pose with transfer ID 0
    send_can_msg(ROBOT_SET_CURRENT_POSE_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::pumpLeftCB(const std_msgs::msg::Bool::SharedPtr msg)
{
    static CanardTransferID transfer_id = 0;
    jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 pumpStatus;
    pumpStatus.status.ID = CAN_PROTOCOL_PUMP_LEFT_ID;
    pumpStatus.status.enabled.value = msg->data;

    size_t buf_size = jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_serialize_(&pumpStatus, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::pumpLeftCB error: Failed serilializing pneumatics_PumpStatus " << res);
        return;
    }

    send_can_msg(ACTION_PUMP_SET_STATUS_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::pumpRightCB(const std_msgs::msg::Bool::SharedPtr msg)
{
    static CanardTransferID transfer_id = 0;
    jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 pumpStatus;
    pumpStatus.status.ID = CAN_PROTOCOL_PUMP_RIGHT_ID;
    pumpStatus.status.enabled.value = msg->data;

    size_t buf_size = jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_serialize_(&pumpStatus, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::pumpRightCB error: Failed serilializing pneumatics_PumpStatus " << res);
        return;
    }

    send_can_msg(ACTION_PUMP_SET_STATUS_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::valveLeftCB(const std_msgs::msg::Bool::SharedPtr msg)
{
    static CanardTransferID transfer_id = 0;

    jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 valveStatus;
    valveStatus.status.ID = CAN_PROTOCOL_VALVE_LEFT_ID;
    valveStatus.status.enabled.value = msg->data;

    size_t buf_size = jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_serialize_(&valveStatus, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::valveLeftCB error: Failed serilializing pneumatics_ValveStatus " << res);
        return;
    }

    send_can_msg(ACTION_VALVE_SET_STATUS_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::valveRightCB(const std_msgs::msg::Bool::SharedPtr msg)
{
    static CanardTransferID transfer_id = 0;

    jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 valveStatus;
    valveStatus.status.ID = CAN_PROTOCOL_VALVE_RIGHT_ID;
    valveStatus.status.enabled.value = msg->data;

    size_t buf_size = jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_serialize_(&valveStatus, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::valveRightCB error: Failed serilializing pneumatics_ValveStatus " << res);
        return;
    }

    send_can_msg(ACTION_VALVE_SET_STATUS_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::sendAdaptPidConfig(std::string side)
{
    static CanardTransferID transfer_id = 0;
    size_t buf_size = jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res;
    if (side == "left")
    {
        res = jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_serialize_(&leftAdaptConfig, buffer, &buf_size);
    }
    else
    {
        res = jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1_serialize_(&rightAdaptConfig, buffer, &buf_size);
    }

    if (res != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::sendAdaptPidConfig error: Failed serilializing motion_AdaptativePIDConfig " << res);
        return;
    }

    send_can_msg(MOTION_SET_ADAPTATIVE_PID_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::sendMotionConfig()
{
    static CanardTransferID transfer_id = 0;

    size_t buf_size = jeroboam_datatypes_actuators_motion_MotionConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_motion_MotionConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_motion_MotionConfig_0_1_serialize_(&motionConfig, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::motionConfigCB error: Failed serilializing motion_MotionConfig " << res);
        return;
    }

    send_can_msg(MOTION_SET_MOTION_CONFIG_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::servoAngleCB(const jrb_msgs::msg::ServoAngle msg)
{
    static CanardTransferID transfer_id = 0;

    jeroboam_datatypes_actuators_servo_ServoAngle_0_1 servoAngle;

    servoAngle.ID = msg.id;
    if(servoAngle.ID==0)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "servoAngle.id = 0");
    }
    servoAngle.angle.radian = msg.radian;

    size_t buf_size = jeroboam_datatypes_actuators_servo_ServoAngle_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_servo_ServoAngle_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_servo_ServoAngle_0_1_serialize_(&servoAngle, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::servoAngleCB error: Failed serilializing servo_ServoAngle " << res);
        return;
    }

    send_can_msg(ACTION_SERVO_SET_ANGLE_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::servoConfigCB(const jrb_msgs::msg::ServoConfig msg)
{
    static CanardTransferID transfer_id = 0;

    jeroboam_datatypes_actuators_servo_ServoConfig_0_1 servoConfig;

    servoConfig.ID = msg.id;
    servoConfig._torque_limit = msg.torque_limit;
    servoConfig.moving_speed = msg.moving_speed;
    servoConfig.pid.pid[0] = msg.pid.pid[0];
    servoConfig.pid.pid[1] = msg.pid.pid[1];
    servoConfig.pid.pid[2] = msg.pid.pid[2];

    if(servoConfig.ID==0)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "servoConfig.id = 0");
    }

    size_t buf_size = jeroboam_datatypes_actuators_servo_ServoConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_servo_ServoConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_servo_ServoConfig_0_1_serialize_(&servoConfig, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::servoConfigCB error: Failed serilializing servo_ServoConfig " << res);
        return;
    }

    send_can_msg(ACTION_SERVO_SET_CONFIG_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::servoRebootCB(const std_msgs::msg::UInt8 msg)
{
    static CanardTransferID transfer_id = 0;

    jeroboam_datatypes_actuators_servo_ServoID_0_1 servoID;

    servoID.ID = msg.data;
    if(servoID.ID==0)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "servoID.id = 0");
    }

    size_t buf_size = jeroboam_datatypes_actuators_servo_ServoID_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_servo_ServoID_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_servo_ServoID_0_1_serialize_(&servoID, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::servoRebootCB error: Failed serilializing servo_ServoID " << res);
        return;
    }

    send_can_msg(ACTION_SERVO_REBOOT_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::servoGenericCommandCB(const jrb_msgs::msg::ServoGenericCommand msg)
{
    static CanardTransferID transfer_id = 0;

    jeroboam_datatypes_actuators_servo_GenericCommand_0_1 command;

    command.id = msg.id;
    command.addr = msg.addr;
    command.data.count = msg.len;
    if(command.id==0)
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "command.id = 0");
    }
    //if(command.addr != 30)
    //{
        RCLCPP_INFO(this->get_logger(), "id=%d / addr=%d",command.id,command.addr);
    //}
    for (int i = 0; i < msg.len; ++i)
    {
        command.data.elements[i] = msg.data[i];
    }

    size_t buf_size = jeroboam_datatypes_actuators_servo_GenericCommand_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_servo_GenericCommand_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_servo_GenericCommand_0_1_serialize_(&command, buffer, &buf_size);
    if (res != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::servoGenericCommandCB error: Failed serilializing servo_GenericCommand " << res);
        return;
    }

    send_can_msg(ACTION_SERVO_GENERIC_COMMAND_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::servoGenericReadCB(const jrb_msgs::msg::ServoGenericRead msg)
{
    static CanardTransferID transfer_id = 0;

    jeroboam_datatypes_actuators_servo_GenericRead_0_1 read;

    read.id = msg.id;
    read.addr = msg.addr;
    read.len = msg.len;

    size_t buf_size = jeroboam_datatypes_actuators_servo_GenericRead_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_servo_GenericRead_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_servo_GenericRead_0_1_serialize_(&read, buffer, &buf_size);

    if (res != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::servoGenericReadCB error: Failed serilializing servo_GenericRead " << res);
        return;
    }

    send_can_request(ACTION_SERVO_GENERIC_READ_ID, CAN_PROTOCOL_ACTION_BOARD_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::motionSpeedCommandCB(const jrb_msgs::msg::MotionSpeedCommand msg)
{
    static CanardTransferID transfer_id = 0;
    jeroboam_datatypes_actuators_motion_SpeedCommand_0_1 command;

    command.left.meter_per_second = msg.left;
    command.right.meter_per_second = msg.right;

    size_t buf_size = jeroboam_datatypes_actuators_motion_SpeedCommand_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_motion_SpeedCommand_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_motion_SpeedCommand_0_1_serialize_(&command, buffer, &buf_size);

    if (res != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed serilializing speed command " << res);
        return;
    }

    send_can_msg(ROBOT_GOAL_SPEEDS_WHEELS_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::turbineSpeedCB(const std_msgs::msg::UInt16 msg)
{
    static CanardTransferID transfer_id = 0;
    jeroboam_datatypes_actuators_pneumatics_TurbineCmd_0_1 command;
    RCLCPP_INFO_STREAM(this->get_logger(), "Turbine Speed CB");
    command.speed = msg.data;

    size_t buf_size = jeroboam_datatypes_actuators_pneumatics_TurbineCmd_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_pneumatics_TurbineCmd_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_pneumatics_TurbineCmd_0_1_serialize_(&command, buffer, &buf_size);

    if (res != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed serilializing speed command " << res);
        return;
    }

    send_can_msg(ACTION_TURBINE_CMD_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::pwmCommandCB (const std_msgs::msg::Float32MultiArray msg)
{
    static CanardTransferID transfer_id = 0;
    jeroboam_datatypes_actuators_motion_SpeedCommand_0_1 command;

    command.left.meter_per_second = msg.data[0];
    command.right.meter_per_second = msg.data[1];

    size_t buf_size = jeroboam_datatypes_actuators_motion_SpeedCommand_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_motion_SpeedCommand_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    int8_t res = jeroboam_datatypes_actuators_motion_SpeedCommand_0_1_serialize_(&command, buffer, &buf_size);

    if (res != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed serilializing speed command " << res);
        return;
    }

    send_can_msg(ROBOT_SET_PWM_WHEELS_ID, &transfer_id, buffer, buf_size);
}