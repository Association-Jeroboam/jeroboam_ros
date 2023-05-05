#include "jrb_strategy2/nodes/condition/is_timer_over_condition.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <chrono>

using namespace BT;

IsTimerOverCondition::IsTimerOverCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  is_timer_over(false)
{
    getInput("sec", sec);
    node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    clock = rclcpp::Clock::make_shared();
    auto callback_group = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false
    );
    callback_group_executor.add_callback_group(callback_group, node->get_node_base_interface());
}

BT::NodeStatus IsTimerOverCondition::tick() {
    if (timer == nullptr && !is_timer_over) {
        timer = rclcpp::create_timer(node, clock, std::chrono::seconds(sec), [this]()
        {
            is_timer_over = true;
            timer->cancel();
        });

        return BT::NodeStatus::FAILURE;
    }

    rclcpp::spin_some(node);

    if (is_timer_over) {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<IsTimerOverCondition>("IsTimerOver");
}
