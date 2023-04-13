#include <string>

#include "jrb_strategy2/nodes/condition/is_match_started_condition.hpp"

IsMatchStartedCondition::IsMatchStartedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  starter_topic("/hardware/starter"),
  started_value(false),
  is_match_started(false)
{
    getInput("starter_topic", starter_topic);
    getInput("started_value", started_value);

    node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false
    );
    callback_group_executor.add_callback_group(callback_group, node->get_node_base_interface());

    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

    qos_profile.depth = 1;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), qos_profile);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group;
    starter_sub = node->create_subscription<std_msgs::msg::Bool>(
        starter_topic,
        qos,
        std::bind(&IsMatchStartedCondition::starterCallback, this, std::placeholders::_1), 
        sub_option
    );
}

BT::NodeStatus IsMatchStartedCondition::tick() {
    callback_group_executor.spin_some();

    if (is_match_started) {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

void IsMatchStartedCondition::starterCallback(std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data == started_value) {
        is_match_started = true;
    } else {
        is_match_started = false;
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<IsMatchStartedCondition>("IsMatchStarted");
}
