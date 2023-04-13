#ifndef IS_MATCH_STARTED_HPP
#define IS_MATCH_STARTED_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/condition_node.h"


class IsMatchStartedCondition : public BT::ConditionNode
{
public:
  IsMatchStartedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsMatchStartedCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "starter_topic", std::string("/hardware/starter"), "Match starter topic"),
      BT::InputPort<bool>(
        "started_value", false, "Boolean value when the starter is pulled. Default to a pull-down."),
    };
  }

private:
  void starterCallback(std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr starter_sub;

  std::string starter_topic;
  bool started_value;
  bool is_match_started;
};

#endif  