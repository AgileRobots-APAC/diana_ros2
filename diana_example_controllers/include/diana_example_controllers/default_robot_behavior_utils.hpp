// Copyright (c) 2023 Franka Robotics GmbH
// Copyright (c) 2026 Agile Robots SE (modifications: adapted for Diana robots)
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

#pragma once

#include <chrono>
#include <future>
#include <memory>

#include <diana_example_controllers/robot_utils.hpp>
#include <diana_msgs/srv/set_full_collision_behavior.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace DefaultRobotBehavior {

const auto kSetDefaultBehaviorTimeout = std::chrono::seconds(5);

inline diana_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr
getDefaultCollisionBehaviorRequest() {
  auto request = std::make_shared<diana_msgs::srv::SetFullCollisionBehavior::Request>();

  request->lower_torque_thresholds_nominal = {
      25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->upper_torque_thresholds_nominal = {
      35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->lower_torque_thresholds_acceleration = {
      25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->upper_torque_thresholds_acceleration = {
      35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->lower_force_thresholds_nominal = {
      30.0, 30.0, 30.0, 25.0, 25.0, 25.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->upper_force_thresholds_nominal = {
      40.0, 40.0, 40.0, 35.0, 35.0, 35.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->lower_force_thresholds_acceleration = {
      30.0, 30.0, 30.0, 25.0, 25.0, 25.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
  request->upper_force_thresholds_acceleration = {
      40.0, 40.0, 40.0, 35.0, 35.0, 35.0};  // NOLINT(cppcoreguidelines-avoid-magic-numbers)

  return request;
}

/**
 * Calls the set_default_behavior service of the hardware, which sets the default joint and
 * Cartesian impedance of the connected robot type. Needed by controllers using the joint/Cartesian
 * position and velocity interfaces.
 *
 * @param[in] node the controller node.
 * @return true if the default behavior was set, false otherwise.
 */
inline bool setDefaultBehavior(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node) {
  auto client = node->create_client<std_srvs::srv::Trigger>("service_server/set_default_behavior");
  if (!client->wait_for_service(kSetDefaultBehaviorTimeout)) {
    RCLCPP_FATAL(node->get_logger(), "Service %s is not available.", client->get_service_name());
    return false;
  }
  auto future_result =
      client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if (future_result.wait_for(kSetDefaultBehaviorTimeout) != std::future_status::ready) {
    RCLCPP_FATAL(node->get_logger(), "Timed out while setting default behavior.");
    return false;
  }

  auto response = future_result.get();
  if (!response->success) {
    RCLCPP_FATAL(node->get_logger(), "Failed to set default behavior: %s",
                 response->message.c_str());
    return false;
  }
  RCLCPP_INFO(node->get_logger(), "%s", response->message.c_str());
  return true;
}

}  // namespace DefaultRobotBehavior
