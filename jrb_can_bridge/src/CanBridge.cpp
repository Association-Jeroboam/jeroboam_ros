#include "CanBridge.hpp"
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

CanBridge::CanBridge()
    : Node("can_bridge"), send_config_enabled(false), init_done(false)
{
}

void CanBridge::init()
{
    // Parameters
    const char* robot_name_env_ptr = std::getenv("ROBOT_NAME");
    std::string robot_name_env = robot_name_env_ptr ? std::string(robot_name_env_ptr) : "";

    this->declare_parameter<std::string>("robot_name", robot_name_env);
    this->get_parameter("robot_name", robot_name);

    if (robot_name != "robotrouge" && robot_name != "robotbleu")
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid 'robot_name' parameter value: %s. Allowed values are 'robotrouge' or 'robotbleu'. Defaulting on robotrouge.", robot_name.c_str());
        robot_name = "robotrouge";
    }

    const auto sides = std::array<std::string, 2>({"left", "right"});
    const auto thresholds = std::array<std::string, 3>({"low", "medium", "high"});
    ros2_utils::floating_point_range default_range = {0.0, 10.0, 0.0001};

    for (auto const &side : sides)
    {
        for (auto const &threshold : thresholds)
        {
            double value;

            ros2_utils::add_parameter((rclcpp::Node &)*this, std::string("pid/" + side + "/" + threshold + "/p"), rclcpp::ParameterValue(0.004), default_range, std::string(side + " left pid motor proportional coef"), std::string(""), false);
            value = this->get_parameter("pid/" + side + "/" + threshold + "/p").as_double();
            setAdaptPidParam(side, threshold, "p", value);

            ros2_utils::add_parameter((rclcpp::Node &)*this, std::string("pid/" + side + "/" + threshold + "/i"), rclcpp::ParameterValue(0.0005), default_range, std::string(side + " left pid motor integral coef"), std::string(""), false);
            value = this->get_parameter("pid/" + side + "/" + threshold + "/i").as_double();
            setAdaptPidParam(side, threshold, "i", value);

            ros2_utils::add_parameter((rclcpp::Node &)*this, std::string("pid/" + side + "/" + threshold + "/d"), rclcpp::ParameterValue(0.0), default_range, std::string(side + " left pid motor derivative coef"), std::string(""), false);
            value = this->get_parameter("pid/" + side + "/" + threshold + "/d").as_double();
            setAdaptPidParam(side, threshold, "d", value);

            ros2_utils::add_parameter((rclcpp::Node &)*this, std::string("pid/" + side + "/" + threshold + "/bias"), rclcpp::ParameterValue(0.0), default_range, std::string(side + " left pid motor bias"), std::string(""), false);
            value = this->get_parameter("pid/" + side + "/" + threshold + "/bias").as_double();
            setAdaptPidParam(side, threshold, "bias", value);

            ros2_utils::add_parameter((rclcpp::Node &)*this, std::string("pid/" + side + "/" + threshold + "/threshold"), rclcpp::ParameterValue(0.01), default_range, std::string(side + " velocity threshlod"), std::string(""), false);
            value = this->get_parameter("pid/" + side + "/" + threshold + "/threshold").as_double();
            setAdaptPidParam(side, threshold, "threshold", value);
        }

        sendAdaptPidConfig(side);
    }

    send_config_enabled = true;

    // Tf
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    static const rclcpp::QoS emgerceny_qos = rclcpp::QoS(1)
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
            .keep_last(1)
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    // Publishers
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 50);

    left_pid_pub = this->create_publisher<jrb_msgs::msg::PIDState>("/hardware/base/pid/left_state", 10);
    right_pid_pub = this->create_publisher<jrb_msgs::msg::PIDState>("/hardware/base/pid/right_state", 10);
    odometry_ticks_pub = this->create_publisher<jrb_msgs::msg::OdometryTicks>("/hardware/base/odometry_ticks", 10);

    servo_generic_read_response_pub = this->create_publisher<jrb_msgs::msg::ServoGenericReadResponse>("/hardware/servo/generic_read_response", 10);
    servo_angle_pub = this->create_publisher<jrb_msgs::msg::ServoAngle>("/hardware/servo/angle", 10);
    
    emergency_pub = this->create_publisher<std_msgs::msg::Bool>("/hardware/emergency/status", emgerceny_qos);

    // Subscribers
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

    motion_config_sub = this->create_subscription<jrb_msgs::msg::MotionConfig>(
        "/hardware/base/motion_config", qos_profile, std::bind(&CanBridge::motionConfigCB, this, std::placeholders::_1));
    motion_speed_command_sub = this->create_subscription<jrb_msgs::msg::MotionSpeedCommand>(
        "/hardware/base/speed_command", qos_profile, std::bind(&CanBridge::motionSpeedCommandCB, this, std::placeholders::_1));

    // Parameters callback
    param_callback_handle = this->add_on_set_parameters_callback(std::bind(&CanBridge::parametersCallback, this, std::placeholders::_1));

    if (robot_name == "robotrouge")
    {
        // Subscribers
        left_pump_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/hardware/pump/left/set_status", 4, std::bind(&CanBridge::pumpLeftCB, this, std::placeholders::_1));
        right_pump_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/hardware/pump/right/set_status", 4, std::bind(&CanBridge::pumpRightCB, this, std::placeholders::_1));

        left_valve_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/hardware/valve/left/set_status", 4, std::bind(&CanBridge::valveLeftCB, this, std::placeholders::_1));
        right_valve_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/hardware/valve/right/set_status", 4, std::bind(&CanBridge::valveRightCB, this, std::placeholders::_1));

        // Publishers
        left_pump_pub = this->create_publisher<std_msgs::msg::Bool>("/hardware/pump/left/status", 10);
        right_pump_pub = this->create_publisher<std_msgs::msg::Bool>("/hardware/pump/right/status", 10);

        left_valve_pub = this->create_publisher<std_msgs::msg::Bool>("/hardware/valve/left/status", 10);
        right_valve_pub = this->create_publisher<std_msgs::msg::Bool>("/hardware/valve/right/status", 10);
    } 
    else if (robot_name == "robotbleu")
    {
        // Subscribers
        turbine_speed_sub = this->create_subscription<std_msgs::msg::UInt16>(
            "/hardware/turbine/speed", 1, std::bind(&CanBridge::turbineSpeedCB, this, std::placeholders::_1));
    }
    else 
    {
        RCLCPP_FATAL(this->get_logger(), "Invalid 'robot_name' parameter value: %s. Allowed values are 'robotrouge' or 'robotbleu'.", robot_name.c_str());
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }


    // CAN messages
    leftAdaptConfig.ID = CAN_PROTOCOL_LEFT_SPEED_PID_ID;
    rightAdaptConfig.ID = CAN_PROTOCOL_RIGHT_SPEED_PID_ID;

    // Timers
    // send_config_timer =
    init_done = true;
}

void CanBridge::setAdaptPidParam(std::string side, std::string threshold, std::string param_name, double value)
{
    jeroboam_datatypes_actuators_motion_AdaptativePIDConfig_0_1 *adaptConfig;
    RCLCPP_INFO(this->get_logger(), "%s %s %s %f", side.c_str(), threshold.c_str(), param_name.c_str(), value);

    if (side == "left")
    {
        adaptConfig = &leftAdaptConfig;
    }
    else
    {
        adaptConfig = &rightAdaptConfig;
    }

    size_t conf_idx = 0;
    if (threshold == "low")
    {
        conf_idx = 0;
    }
    else if (threshold == "medium")
    {
        conf_idx = 1;
    }
    else
    {
        conf_idx = 2;
    }

    if (param_name == "bias")
    {
        adaptConfig->configs[conf_idx].bias = value;
    }
    else if (param_name == "threshold")
    {
        adaptConfig->thresholds[conf_idx] = value;
    }
    else
    {
        size_t pid_idx = 0;
        if (param_name == "p")
        {
            pid_idx = 0;
        }
        else if (param_name == "i")
        {
            pid_idx = 1;
        }
        else
        {
            pid_idx = 2;
        }

        adaptConfig->configs[conf_idx].pid[pid_idx] = value;
    }
}

rcl_interfaces::msg::SetParametersResult CanBridge::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    for (const auto &parameter : parameters)
    {
        auto name = parameter.get_name();

        if (name.rfind("pid/", 0) == 0)
        {
            std::string side, threshold, param_name;
            std::stringstream s(name);
            std::string part;
            std::vector<std::string> parts;

            while (std::getline(s, part, '/'))
            {
                parts.push_back(part);
            }

            side = parts[1];
            threshold = parts[2];
            param_name = parts[3];
            auto value = parameter.as_double();

            setAdaptPidParam(side, threshold, param_name, value);
        }
    }
    if (send_config_enabled)
    {
        sendAdaptPidConfig("left");
        sendAdaptPidConfig("right");
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    RCLCPP_INFO(this->get_logger(), "Params updated");

    return result;
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
    if (!success)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("CanBridge"), "Queue push failed\n");
    }
    (*transferID)++;
}

void CanBridge::send_can_request(CanardPortID portID, CanardNodeID destID, CanardTransferID *transferID, void *buffer, size_t buf_size)
{
    CanardTransferMetadata metadata;
    metadata.priority = CanardPriorityNominal;
    metadata.transfer_kind = CanardTransferKindRequest;
    metadata.port_id = portID;
    metadata.remote_node_id = destID;
    metadata.transfer_id = *transferID;

    bool success = TxThread::pushQueue(&metadata, buf_size, buffer);
    if (!success)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("CanBridge"), "Queue push failed\n");
    }
    (*transferID)++;
}
