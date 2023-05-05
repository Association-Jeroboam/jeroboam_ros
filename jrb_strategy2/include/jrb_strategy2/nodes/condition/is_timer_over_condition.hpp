
#ifndef IS_TIMER_OVER_HPP
#define IS_TIMER_OVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

class IsTimerOverCondition : public BT::ConditionNode
{
public:
  IsTimerOverCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsTimerOverCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>(
        "sec", "Timer seconds")
    };
  }

private:
  rclcpp::Node::SharedPtr node;
  rclcpp::Clock::SharedPtr clock;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor;
  rclcpp::TimerBase::SharedPtr timer;

  int sec;
  bool is_timer_over;
};

#endif  