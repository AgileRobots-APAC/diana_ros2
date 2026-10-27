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

#include "diana_hardware/diana_param_service_server.hpp"

#include <algorithm>

#include "diana_hardware/default_robot_behavior.hpp"

namespace diana_hardware {

DianaParamServiceServer::DianaParamServiceServer(const rclcpp::NodeOptions& options,
                                                 std::shared_ptr<Robot> robot)
    : rclcpp::Node("service_server", options), robot_(std::move(robot)) {
  set_joint_stiffness_service_ = create_service<diana_msgs::srv::SetJointStiffness>(
      "~/set_joint_stiffness",
      [this](const std::shared_ptr<diana_msgs::srv::SetJointStiffness::Request>& request,
             const std::shared_ptr<diana_msgs::srv::SetJointStiffness::Response>& response) {
        setJointStiffnessCallback(request, response);
      });

  set_cartesian_stiffness_service_ = create_service<diana_msgs::srv::SetCartesianStiffness>(
      "~/set_cartesian_stiffness",
      [this](const std::shared_ptr<diana_msgs::srv::SetCartesianStiffness::Request>& request,
             const std::shared_ptr<diana_msgs::srv::SetCartesianStiffness::Response>& response) {
        setCartesianStiffnessCallback(request, response);
      });

  set_tcp_frame_service_ = create_service<diana_msgs::srv::SetTCPFrame>(
      "~/set_tcp_frame",
      [this](const std::shared_ptr<diana_msgs::srv::SetTCPFrame::Request>& request,
             const std::shared_ptr<diana_msgs::srv::SetTCPFrame::Response>& response) {
        setTCPFrameCallback(request, response);
      });

  set_stiffness_frame_service_ = create_service<diana_msgs::srv::SetStiffnessFrame>(
      "~/set_stiffness_frame",
      [this](const std::shared_ptr<diana_msgs::srv::SetStiffnessFrame::Request>& request,
             const std::shared_ptr<diana_msgs::srv::SetStiffnessFrame::Response>& response) {
        setStiffnessFrameCallback(request, response);
      });

  set_force_torque_collision_behavior_service_ =
      create_service<diana_msgs::srv::SetForceTorqueCollisionBehavior>(
          "~/set_force_torque_collision_behavior",
          [this](const std::shared_ptr<diana_msgs::srv::SetForceTorqueCollisionBehavior::Request>&
                     request,
                 const std::shared_ptr<diana_msgs::srv::SetForceTorqueCollisionBehavior::Response>&
                     response) { setForceTorqueCollisionBehaviorCallback(request, response); });

  set_full_collision_behavior_service_ = create_service<diana_msgs::srv::SetFullCollisionBehavior>(
      "~/set_full_collision_behavior",
      [this](const std::shared_ptr<diana_msgs::srv::SetFullCollisionBehavior::Request>& request,
             const std::shared_ptr<diana_msgs::srv::SetFullCollisionBehavior::Response>& response) {
        setFullCollisionBehaviorCallback(request, response);
      });

  set_load_service_ = create_service<diana_msgs::srv::SetLoad>(
      "~/set_load", [this](const std::shared_ptr<diana_msgs::srv::SetLoad::Request>& request,
                           const std::shared_ptr<diana_msgs::srv::SetLoad::Response>& response) {
        setLoadCallback(request, response);
      });

  set_default_behavior_service_ = create_service<std_srvs::srv::Trigger>(
      "~/set_default_behavior",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
             const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
        setDefaultBehaviorCallback(request, response);
      });

  RCLCPP_INFO(get_logger(), "Service started");
}

void DianaParamServiceServer::setJointStiffnessCallback(
    const diana_msgs::srv::SetJointStiffness::Request::SharedPtr& request,
    const diana_msgs::srv::SetJointStiffness::Response::SharedPtr& response) {
  auto set_joint_stiffness_function =
      [&](const diana_msgs::srv::SetJointStiffness::Request::SharedPtr& request) {
        robot_->setJointStiffness(request);
      };
  setGenericRobotParam<diana_msgs::srv::SetJointStiffness::Request::SharedPtr,
                       diana_msgs::srv::SetJointStiffness::Response::SharedPtr>(
      set_joint_stiffness_function, request, response);
}

void DianaParamServiceServer::setCartesianStiffnessCallback(
    const diana_msgs::srv::SetCartesianStiffness::Request::SharedPtr& request,
    const diana_msgs::srv::SetCartesianStiffness::Response::SharedPtr& response) {
  auto set_cartesian_stiffness_function =
      [&](const diana_msgs::srv::SetCartesianStiffness::Request::SharedPtr& request) {
        robot_->setCartesianStiffness(request);
      };
  setGenericRobotParam<diana_msgs::srv::SetCartesianStiffness::Request::SharedPtr,
                       diana_msgs::srv::SetCartesianStiffness::Response::SharedPtr>(
      set_cartesian_stiffness_function, request, response);
}

void DianaParamServiceServer::setTCPFrameCallback(
    const diana_msgs::srv::SetTCPFrame::Request::SharedPtr& request,
    const diana_msgs::srv::SetTCPFrame::Response::SharedPtr& response) {
  auto set_tcp_frame_function =
      [&](const diana_msgs::srv::SetTCPFrame::Request::SharedPtr& request) {
        robot_->setTCPFrame(request);
      };
  setGenericRobotParam<diana_msgs::srv::SetTCPFrame::Request::SharedPtr,
                       diana_msgs::srv::SetTCPFrame::Response::SharedPtr>(set_tcp_frame_function,
                                                                          request, response);
}

void DianaParamServiceServer::setStiffnessFrameCallback(
    const diana_msgs::srv::SetStiffnessFrame::Request::SharedPtr& request,
    const diana_msgs::srv::SetStiffnessFrame::Response::SharedPtr& response) {
  auto set_stiffness_frame_function =
      [&](const diana_msgs::srv::SetStiffnessFrame::Request::SharedPtr& request) {
        robot_->setStiffnessFrame(request);
      };
  setGenericRobotParam<diana_msgs::srv::SetStiffnessFrame::Request::SharedPtr,
                       diana_msgs::srv::SetStiffnessFrame::Response::SharedPtr>(
      set_stiffness_frame_function, request, response);
}

void DianaParamServiceServer::setForceTorqueCollisionBehaviorCallback(
    const diana_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& request,
    const diana_msgs::srv::SetForceTorqueCollisionBehavior::Response::SharedPtr& response) {
  auto set_force_torque_collision_behavior_function =
      [&](const diana_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& request) {
        robot_->setForceTorqueCollisionBehavior(request);
      };
  setGenericRobotParam<diana_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr,
                       diana_msgs::srv::SetForceTorqueCollisionBehavior::Response::SharedPtr>(
      set_force_torque_collision_behavior_function, request, response);
}

void DianaParamServiceServer::setFullCollisionBehaviorCallback(
    const diana_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& request,
    const diana_msgs::srv::SetFullCollisionBehavior::Response::SharedPtr& response) {
  auto set_full_collision_behavior_function =
      [&](const diana_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& request) {
        robot_->setFullCollisionBehavior(request);
      };
  setGenericRobotParam<diana_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr,
                       diana_msgs::srv::SetFullCollisionBehavior::Response::SharedPtr>(
      set_full_collision_behavior_function, request, response);
}

void DianaParamServiceServer::setLoadCallback(
    const diana_msgs::srv::SetLoad::Request::SharedPtr& request,
    const diana_msgs::srv::SetLoad::Response::SharedPtr& response) {
  auto set_load_function = [&](const diana_msgs::srv::SetLoad::Request::SharedPtr& request) {
    robot_->setLoad(request);
  };
  setGenericRobotParam<diana_msgs::srv::SetLoad::Request::SharedPtr,
                       diana_msgs::srv::SetLoad::Response::SharedPtr>(set_load_function, request,
                                                                      response);
}

void DianaParamServiceServer::setDefaultBehaviorCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr& /*request*/,
    const std_srvs::srv::Trigger::Response::SharedPtr& response) {
  response->success = false;
  auto robot_type = robot_->getRobotType();
  if (robot_type.empty()) {
    response->message = "Failed to get the robot type of the connected robot.";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  auto default_behavior = getDefaultRobotBehavior(robot_type);
  if (!default_behavior) {
    response->message = "No default behavior defined for robot type '" + robot_type + "'.";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  auto joint_stiffness_request = std::make_shared<diana_msgs::srv::SetJointStiffness::Request>();
  std::copy(default_behavior->joint_stiffness.cbegin(), default_behavior->joint_stiffness.cend(),
            joint_stiffness_request->joint_stiffness.begin());
  auto cartesian_stiffness_request =
      std::make_shared<diana_msgs::srv::SetCartesianStiffness::Request>();
  std::copy(default_behavior->cartesian_stiffness.cbegin(),
            default_behavior->cartesian_stiffness.cend(),
            cartesian_stiffness_request->cartesian_stiffness.begin());

  try {
    robot_->setJointStiffness(joint_stiffness_request);
    robot_->setCartesianStiffness(cartesian_stiffness_request);
    response->success = true;
    response->message = "Default behavior set for robot type '" + robot_type + "'.";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  } catch (const franka::Exception& e) {
    RCLCPP_ERROR(get_logger(), "Exception during setting default behavior: %s", e.what());
    response->message = e.what();
  }
}

}  // namespace diana_hardware
