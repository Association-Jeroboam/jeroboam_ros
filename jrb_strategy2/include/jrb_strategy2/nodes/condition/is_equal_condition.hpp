#ifndef IS_EQUAL_HPP
#define IS_EQUAL_HPP

#include <mutex>

#include "behaviortree_cpp_v3/condition_node.h"

class IsEqualCondition : public BT::ConditionNode
{
public:
  IsEqualCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsEqualCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "type", "Type of the operands. Must be one of C++ primitive types or string"),
      BT::InputPort<std::string>(
        "operand1", "First operand"),
      BT::InputPort<std::string>(
        "operand2", "Second operand"),
      BT::InputPort<double>(
        "epsilon", 1e-4, "Difference precision used in floating point comparison")
    };
  }

private:
  static std::array<std::string, 7> supported_types;
  std::string type;
  std::string operand1;
  std::string operand2;
  double epsilon;
};

#endif  