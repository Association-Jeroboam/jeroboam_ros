#include "jrb_strategy2/nodes/condition/is_equal_condition.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <cmath>

using namespace BT;

std::array<std::string, 7> IsEqualCondition::supported_types = {"string", "bool", "char", "int", "long", "float", "double"};

IsEqualCondition::IsEqualCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{

}

BT::NodeStatus IsEqualCondition::tick() {
    getInput("type", type);
    getInput("operand1", operand1);
    getInput("operand2", operand2);
    getInput("epsilon", epsilon);

    if (std::find(supported_types.begin(), supported_types.end(), type) == supported_types.end()) {
        throw BT::RuntimeError("Unkown primitive type : " + type);
    }

    if ((type == "string" || type == "char") && operand1 == operand2) {
        return BT::NodeStatus::SUCCESS;
    }

    if (type == "bool" && convertFromString<bool>(operand1) == convertFromString<bool>(operand2)) {
        return BT::NodeStatus::SUCCESS;
    }

    if (type == "int" && convertFromString<int>(operand1) == convertFromString<int>(operand2)) {
            return BT::NodeStatus::SUCCESS;
    } 

    if (type == "long" && convertFromString<long>(operand1) == convertFromString<long>(operand2)) {
            return BT::NodeStatus::SUCCESS;
    } 

    if (type == "float" && std::fabs(convertFromString<float>(operand1) - convertFromString<float>(operand2)) < epsilon) {
            return BT::NodeStatus::SUCCESS;
    } 

    if (type == "double" && std::fabs(convertFromString<double>(operand1) - convertFromString<double>(operand2)) < epsilon) {
            return BT::NodeStatus::SUCCESS;
    } 

    return BT::NodeStatus::FAILURE;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<IsEqualCondition>("IsEqual");
}
