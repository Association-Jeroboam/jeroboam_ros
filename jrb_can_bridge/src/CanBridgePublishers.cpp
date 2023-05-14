#include "CanBridge.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

void CanBridge::initPubs() {
     static const rclcpp::QoS emgerceny_qos = rclcpp::QoS(1)
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
            .keep_last(1)
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 50);

    left_pid_pub = this->create_publisher<jrb_msgs::msg::PIDState>("/hardware/base/pid/left_state", 10);
    right_pid_pub = this->create_publisher<jrb_msgs::msg::PIDState>("/hardware/base/pid/right_state", 10);
    linear_pid_pub = this->create_publisher<jrb_msgs::msg::PIDState>("/hardware/base/pid/linear_state", 10);
    angular_pid_pub = this->create_publisher<jrb_msgs::msg::PIDState>("/hardware/base/pid/angular_state", 10);
    odometry_ticks_pub = this->create_publisher<jrb_msgs::msg::OdometryTicks>("/hardware/base/odometry_ticks", 10);

    servo_generic_read_response_pub = this->create_publisher<jrb_msgs::msg::ServoGenericReadResponse>("/hardware/servo/generic_read_response", 10);
    servo_angle_pub = this->create_publisher<jrb_msgs::msg::ServoAngle>("/hardware/servo/angle", 10);
    
    emergency_pub = this->create_publisher<std_msgs::msg::Bool>("/hardware/emergency/status", emgerceny_qos);

    if (robot_name == "robotrouge")
    {
        left_pump_pub = this->create_publisher<std_msgs::msg::Bool>("/hardware/pump/left/status", 10);
        right_pump_pub = this->create_publisher<std_msgs::msg::Bool>("/hardware/pump/right/status", 10);

        left_valve_pub = this->create_publisher<std_msgs::msg::Bool>("/hardware/valve/left/status", 10);
        right_valve_pub = this->create_publisher<std_msgs::msg::Bool>("/hardware/valve/right/status", 10);
    } 
    else if (robot_name == "robotbleu")
    {
        // None yet.
    }

}

void CanBridge::publishRobotCurrentState(jeroboam_datatypes_sensors_odometry_State2D_1_0 * state)
{
    /**
     * Publish on the odom topic
     */
    auto state_msg = nav_msgs::msg::Odometry();

    state_msg.header.stamp = get_clock()->now();
    state_msg.header.frame_id = "odom";
    state_msg.child_frame_id = "base_link";

    state_msg.pose.pose.position.x = state->pose.x.meter;
    state_msg.pose.pose.position.y = state->pose.y.meter;

    // This is supposed to work... but has to be tested
    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(0,0, state->pose.theta.radian);
    state_msg.pose.pose.orientation = tf2::toMsg(tf2_quat); 

    state_msg.twist.twist.linear.x = state->twist.linear.meter_per_second;

    state_msg.twist.twist.angular.z = state->twist.angular.radian_per_second;

    odom_pub->publish(state_msg);

    /**
     * Publish the transform
     */
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = state_msg.header.stamp;
    transform.header.frame_id = state_msg.header.frame_id;
    transform.child_frame_id = state_msg.child_frame_id;
    transform.transform.translation.x = state_msg.pose.pose.position.x;
    transform.transform.translation.y = state_msg.pose.pose.position.y;
    transform.transform.translation.z = state_msg.pose.pose.position.z;
    transform.transform.rotation = state_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform);
}

void CanBridge::publishLeftPIDState(jeroboam_datatypes_actuators_motion_PIDState_0_1 *pid)
{
    auto pid_msg = jrb_msgs::msg::PIDState();
    pid_msg.output = pid->output;
    pid_msg.setpoint = pid->setpoint;
    pid_msg.error = pid->_error;

    left_pid_pub->publish(pid_msg);
}

void CanBridge::publishRightPIDState(jeroboam_datatypes_actuators_motion_PIDState_0_1 *pid)
{
    auto pid_msg = jrb_msgs::msg::PIDState();
    pid_msg.output = pid->output;
    pid_msg.setpoint = pid->setpoint;
    pid_msg.error = pid->_error;

    right_pid_pub->publish(pid_msg);
}

void CanBridge::publishLinearPIDState(jeroboam_datatypes_actuators_motion_PIDState_0_1 *pid)
{
    auto pid_msg = jrb_msgs::msg::PIDState();
    pid_msg.output = pid->output;
    pid_msg.setpoint = pid->setpoint;
    pid_msg.error = pid->_error;

    linear_pid_pub->publish(pid_msg);
}

void CanBridge::publishAngularPIDState(jeroboam_datatypes_actuators_motion_PIDState_0_1 *pid)
{
    auto pid_msg = jrb_msgs::msg::PIDState();
    pid_msg.output = pid->output;
    pid_msg.setpoint = pid->setpoint;
    pid_msg.error = pid->_error;

    angular_pid_pub->publish(pid_msg);
}

void CanBridge::publishLeftPumpStatus(jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 *status)
{
    auto status_msg = std_msgs::msg::Bool();
    status_msg.data = status->status.enabled.value;

    left_pump_pub->publish(status_msg);
}
void CanBridge::publishRightPumpStatus(jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 *status)
{
    auto status_msg = std_msgs::msg::Bool();
    status_msg.data = status->status.enabled.value;

    right_pump_pub->publish(status_msg);
}
void CanBridge::publishLeftValveStatus(jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 *status)
{
    auto status_msg = std_msgs::msg::Bool();
    status_msg.data = status->status.enabled.value;

    left_valve_pub->publish(status_msg);
}
void CanBridge::publishRightValveStatus(jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 *status)
{
    auto status_msg = std_msgs::msg::Bool();
    status_msg.data = status->status.enabled.value;

    right_valve_pub->publish(status_msg);
}

void CanBridge::publishServoGenericReadResponse(jeroboam_datatypes_actuators_servo_GenericReadResponse_0_1 *response)
{
    (void) response;
    RCLCPP_ERROR_STREAM(this->get_logger(), "BROKEN DOES NOT WORK (yet.)");
    // auto servo_response = jrb_msgs::msg::ServoGenericReadResponse();

    // if(response._tag_ == CAN_PROTOCOL_RESPONSE_SUCCESS) {
    // RCLCPP_ERROR_STREAM(this->get_logger(), "count %lu\r\n", response.data.count);
    // RCLCPP_ERROR_STREAM(this->get_logger(), "count %lu", response->data.count);
    // for(size_t i = 0; i < response->data.count; i++){
    //        servo_response.data[i] = response->data.elements[i];
    //}
    // servo_response.len = response->data.count;
    //  servo_response.len = 1;
    //} else if(response._tag_ == CAN_PROTOCOL_RESPONSE_ERROR){
    //    RCLCPP_ERROR_STREAM(this->get_logger(), "tag = %i\r\n", response._tag_);
    //    RCLCPP_ERROR_STREAM(this->get_logger(), "error = %u\r\n", response.err_code);
    //    servo_response.len = 0;
    //} else {
    //    RCLCPP_ERROR_STREAM(this->get_logger(), "unhandled tag %u\r\n", response._tag_);;
    //}

    // servo_generic_read_response_pub->publish(servo_response);
}

void CanBridge::publishServoAngle(jeroboam_datatypes_actuators_servo_ServoAngle_0_1 *servoAngle)
{
    auto servo_angle_msg = jrb_msgs::msg::ServoAngle();
    servo_angle_msg.id = servoAngle->ID;
    servo_angle_msg.radian = servoAngle->angle.radian;
    servo_angle_pub->publish(servo_angle_msg);
}

void CanBridge::publishOdometryTicks(jeroboam_datatypes_sensors_odometry_OdometryTicks_0_1 *odometryTicks)
{
    auto ticks = jrb_msgs::msg::OdometryTicks();
    ticks.left = odometryTicks->left;
    ticks.right = odometryTicks->right;
    odometry_ticks_pub->publish(ticks);
}


void CanBridge::publishEmergencyStop(bool * emergency) {
    static bool first_call = true;
    static bool last_emergency = false;
    std_msgs::msg::Bool emergency_msg;

    if((last_emergency != *emergency) || first_call) {
        first_call = false;
        last_emergency = *emergency;
        emergency_msg.data = *emergency;
        emergency_pub->publish(emergency_msg);
    }    
}
