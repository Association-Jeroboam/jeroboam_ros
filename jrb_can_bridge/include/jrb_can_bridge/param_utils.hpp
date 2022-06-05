
   
// Copyright (c) 2018 Intel Corporation
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

#ifndef ROS2_UTILS__PARAM_UTILS_HPP_
#define ROS2_UTILS__PARAM_UTILS_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace ros2_utils
{

typedef struct
{
  double from_value;
  double to_value;
  double step;
} floating_point_range;

typedef struct
{
  int from_value;
  int to_value;
  int step;
} integer_range;

// Declare a parameter that has no integer or floating point range constraints
void add_parameter(
  rclcpp::Node & node,
  const std::string & name,
  const rclcpp::ParameterValue & default_value,
  const std::string & description = "",
  const std::string & additional_constraints = "",
  bool read_only = false);

// Declare a parameter that has a floating point range constraint
void add_parameter(
  rclcpp::Node & node,
  const std::string & name,
  const rclcpp::ParameterValue & default_value,
  const floating_point_range fp_range,
  const std::string & description = "",
  const std::string & additional_constraints = "",
  bool read_only = false);

// Declare a parameter that has an integer range constraint
void add_parameter(
  rclcpp::Node & node,
  const std::string & name,
  const rclcpp::ParameterValue & default_value,
  const integer_range int_range,
  const std::string & description = "",
  const std::string & additional_constraints = "",
  bool read_only = false);

}  // namespace ros2_utils

#endif  // ROS2_UTILS__PARAM_UTILS_HPP_