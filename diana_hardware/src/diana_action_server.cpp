// Copyright (c) 2024 Franka Robotics GmbH
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

#include "diana_hardware/diana_action_server.hpp"

namespace diana_hardware {

ActionServer::ActionServer(const rclcpp::NodeOptions& options, std::shared_ptr<Robot> robot)
    : rclcpp::Node("action_server", options), robot_(robot) {
  error_recovery_action_server_ = rclcpp_action::create_server<diana_msgs::action::ErrorRecovery>(
      this, "~/error_recovery",
      [](auto /*uuid*/, auto /*goal*/) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](const auto& /*goal_handle*/) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const auto& goal_handle) { errorRecoveryAction(goal_handle); });

  RCLCPP_INFO(get_logger(), "Action server started");
}

auto ActionServer::errorRecoveryAction(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<diana_msgs::action::ErrorRecovery>>&
        goal_handle) -> void {
  auto result = std::make_shared<diana_msgs::action::ErrorRecovery::Result>();
  try {
    robot_->automaticErrorRecovery();
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Automatic recovery succeeded");
  } catch (const franka::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during automatic error recovery: %s", e.what());
    goal_handle->abort(result);
  }
}

}  // namespace diana_hardware
