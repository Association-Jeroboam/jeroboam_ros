#include "CanBridge.hpp"
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

void CanBridge::robot_twist_goal_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (msg == nullptr)
    {
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
    if (result != NUNAVUT_SUCCESS)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CanBridge::robot_twist_goal_cb error: Failed serializing cartesian_Twist " << result);
    }

    send_can_msg(ROBOT_TWIST_GOAL_ID, &transfer_id, buffer, buf_size);
}

void CanBridge::initialpose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
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

void CanBridge::motionConfigCB(const jrb_msgs::msg::MotionConfig msg)
{
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
