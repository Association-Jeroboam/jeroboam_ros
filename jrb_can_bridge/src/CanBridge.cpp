#include "CanBridge.hpp"
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

using namespace std::chrono_literals;

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

    ros2_utils::add_parameter((rclcpp::Node &)*this, std::string("wheel_base"), rclcpp::ParameterValue(0.258), (ros2_utils::floating_point_range){0.0, 0.5, 0.0}, std::string("wheel base"), std::string(""), false);
    ros2_utils::add_parameter((rclcpp::Node &)*this, std::string("wheel_radius/left"), rclcpp::ParameterValue(0.02825), (ros2_utils::floating_point_range){0.0, 0.1, 0.0}, std::string("left wheel redius"), std::string(""), false);
    ros2_utils::add_parameter((rclcpp::Node &)*this, std::string("wheel_radius/right"), rclcpp::ParameterValue(0.02825), (ros2_utils::floating_point_range){0.0, 0.1, 0.0}, std::string("right wheel radius"), std::string(""), false);
    ros2_utils::add_parameter((rclcpp::Node &)*this, std::string("max_linear_speed"), rclcpp::ParameterValue(0.5), (ros2_utils::floating_point_range){0.1, 2.0, 0.01}, std::string("max linear speed"), std::string(""), false);
    ros2_utils::add_parameter((rclcpp::Node &)*this, std::string("max_angular_speed"), rclcpp::ParameterValue(90.0), (ros2_utils::floating_point_range){1.0, 720.0, 1.0}, std::string("max angular speed in rad/s"), std::string(""), false);
    ros2_utils::add_parameter((rclcpp::Node &)*this, std::string("max_linear_acceleration"), rclcpp::ParameterValue(0.5), (ros2_utils::floating_point_range){0.1, 5.0, 0.01}, std::string("max linear acceleration"), std::string(""), false);
    ros2_utils::add_parameter((rclcpp::Node &)*this, std::string("max_angular_acceleration"), rclcpp::ParameterValue(90.0), (ros2_utils::floating_point_range){1.0, 1440.0, 0.1}, std::string("max angular acceleration in rad/s^2"), std::string(""), false);

    motionConfig.wheel_base.meter = this->get_parameter("wheel_base").as_double();
    motionConfig.left_wheel_radius.meter = this->get_parameter("wheel_radius/left").as_double();
    motionConfig.right_wheel_radius.meter = this->get_parameter("wheel_radius/right").as_double();
    motionConfig.maxLinearSpeed.meter_per_second = this->get_parameter("max_linear_speed").as_double();
    motionConfig.maxAngularSpeed.radian_per_second = this->get_parameter("max_angular_speed").as_double() * M_PI / 180.0;
    motionConfig.maxLinearAccl.meter_per_second_per_second = this->get_parameter("max_linear_acceleration").as_double();
    motionConfig.maxAngularAccl.radian_per_second_per_second = this->get_parameter("max_angular_acceleration").as_double() * M_PI / 180.0;

    // TODO: send reboot cmd instead of this to reset transfer_id
    sendMotionConfig();
    sendMotionConfig();

    send_config_enabled = true;

    // Tf
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Publishers
    initPubs();

    // Subscribers
    initSubs();

    // Parameters callback
    param_callback_handle = this->add_on_set_parameters_callback(std::bind(&CanBridge::parametersCallback, this, std::placeholders::_1));

    if (robot_name != "robotrouge" && robot_name != "robotbleu")
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
    bool leftPidParamModified = false;
    bool rightPidParamModified = false;
    bool motionConfigParamModified = false;
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

            if (side == "left" && !leftPidParamModified) {
                leftPidParamModified = true;
            }

            if (side == "right" && !rightPidParamModified) {
                rightPidParamModified = true;
            }
        } else {
            motionConfigParamModified = true;

            if (name == "wheel_base") {
                motionConfig.wheel_base.meter = parameter.as_double();
            } else if (name == "wheel_radius/left") {
                motionConfig.left_wheel_radius.meter = parameter.as_double();
            } else if (name == "wheel_radius/right") {
                motionConfig.right_wheel_radius.meter = parameter.as_double();
            } else if (name == "max_linear_speed") {
                motionConfig.maxLinearSpeed.meter_per_second = parameter.as_double();
            } else if (name == "max_angular_speed") {
                motionConfig.maxAngularSpeed.radian_per_second = parameter.as_double();
            } else if (name == "max_linear_acceleration") {
                motionConfig.maxLinearAccl.meter_per_second_per_second = parameter.as_double();
            } else if (name == "max_angular_acceleration") {
                motionConfig.maxAngularAccl.radian_per_second_per_second = parameter.as_double();
            }
        }
    }
    
    if (send_config_enabled) {
        if (leftPidParamModified) {
            sendAdaptPidConfig("left");
        }

        if (rightPidParamModified) {
            sendAdaptPidConfig("right");
        }

        if (motionConfigParamModified) {
            sendMotionConfig();
        }
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
