// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "jrb_can_bridge/param_utils.hpp"

#include <memory>
#include <string>

namespace ros2_utils
{

void
add_parameter(
  rclcpp::Node & node,
  const std::string & name,
  const rclcpp::ParameterValue & default_value,
  const std::string & description,
  const std::string & additional_constraints,
  bool read_only)
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

  descriptor.name = name;
  descriptor.description = description;
  descriptor.additional_constraints = additional_constraints;
  descriptor.read_only = read_only;

  node.declare_parameter(descriptor.name, default_value, descriptor);
}

void
add_parameter(
  rclcpp::Node & node,
  const std::string & name,
  const rclcpp::ParameterValue & default_value,
  const floating_point_range fp_range,
  const std::string & description,
  const std::string & additional_constraints,
  bool read_only)
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

  descriptor.name = name;
  descriptor.description = description;
  descriptor.additional_constraints = additional_constraints;
  descriptor.read_only = read_only;
  descriptor.floating_point_range.resize(1);
  descriptor.floating_point_range[0].from_value = fp_range.from_value;
  descriptor.floating_point_range[0].to_value = fp_range.to_value;
  descriptor.floating_point_range[0].step = fp_range.step;

  node.declare_parameter(descriptor.name, default_value, descriptor);
}

void
add_parameter(
  rclcpp::Node & node,
  const std::string & name,
  const rclcpp::ParameterValue & default_value,
  const integer_range int_range,
  const std::string & description,
  const std::string & additional_constraints,
  bool read_only)
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

  descriptor.name = name;
  descriptor.description = description;
  descriptor.additional_constraints = additional_constraints;
  descriptor.read_only = read_only;
  descriptor.integer_range.resize(1);
  descriptor.integer_range[0].from_value = int_range.from_value;
  descriptor.integer_range[0].to_value = int_range.to_value;
  descriptor.integer_range[0].step = int_range.step;

  node.declare_parameter(descriptor.name, default_value, descriptor);
}

}  // namespace ros2_utils