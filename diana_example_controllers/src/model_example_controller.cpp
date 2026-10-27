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

#include <diana_example_controllers/model_example_controller.hpp>

#include <chrono>
#include <memory>
#include <string>

#include "diana_example_controllers/robot_utils.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

namespace diana_example_controllers {

controller_interface::CallbackReturn ModelExampleController::on_init() {
  try {
    // robot_type is forwarded by the bringup launch file through the
    // controller_manager node parameters. The default is intentionally empty
    // so that no robot model is silently assumed; on_configure() resolves it
    // from the loaded robot_description when it was not provided.
    auto_declare<std::string>("robot_type", "");
    auto_declare<std::string>("arm_prefix", "");
    if (!get_node()->get_parameter("robot_type", robot_type_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to get robot_type parameter");
      get_node()->shutdown();
      return CallbackReturn::ERROR;
    }
    if (!get_node()->get_parameter("arm_prefix", arm_prefix_)) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to get arm_prefix parameter");
      get_node()->shutdown();
      return CallbackReturn::ERROR;
    }
    arm_prefix_ = arm_prefix_.empty() ? "" : arm_prefix_ + "_";
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ModelExampleController::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration ModelExampleController::state_interface_configuration()
    const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& diana_robot_model_name : diana_robot_model_->get_state_interface_names()) {
    state_interfaces_config.names.push_back(diana_robot_model_name);
  }
  return state_interfaces_config;
}

std::string ModelExampleController::getRobotNameFromDescription() const {
  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  if (!parameters_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Parameter service of robot_state_publisher is not available, cannot resolve "
                "robot_type from robot_description.");
    return {};
  }

  auto future = parameters_client->get_parameters({"robot_description"});
  const auto result = future.get();
  if (result.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "Failed to get robot_description parameter.");
    return {};
  }

  return robot_utils::getRobotNameFromDescription(result[0].value_to_string(),
                                                  get_node()->get_logger());
}

controller_interface::CallbackReturn ModelExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  const std::string robot_name_from_description = getRobotNameFromDescription();

  if (robot_type_.empty()) {
    robot_type_ = robot_name_from_description;
  } else if (!robot_name_from_description.empty() && robot_name_from_description != robot_type_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "robot_type parameter '%s' does not match the loaded robot_description '%s'; "
                 "the hardware interfaces are named after the loaded description.",
                 robot_type_.c_str(), robot_name_from_description.c_str());
    return CallbackReturn::FAILURE;
  }

  if (robot_type_.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "robot_type is empty and could not be resolved from robot_description. Pass "
                 "robot_type:=diana3|diana7|diana7g2 to the bringup launch file.");
    return CallbackReturn::FAILURE;
  }

  diana_robot_model_ = std::make_unique<diana_semantic_components::DianaRobotModel>(
      diana_semantic_components::DianaRobotModel(
          arm_prefix_ + robot_type_ + "/" + k_robot_model_interface_name,
          arm_prefix_ + robot_type_ + "/" + k_robot_state_interface_name));

  RCLCPP_DEBUG(get_node()->get_logger(), "configured successfully");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ModelExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  diana_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ModelExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  diana_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ModelExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  std::array<double, 49> mass = diana_robot_model_->getMassMatrix();
  std::array<double, 7> coriolis = diana_robot_model_->getCoriolisForceVector();
  std::array<double, 7> gravity = diana_robot_model_->getGravityForceVector();
  std::array<double, 16> pose = diana_robot_model_->getPoseMatrix(franka::Frame::kJoint4);
  std::array<double, 42> joint4_body_jacobian_wrt_joint4 =
      diana_robot_model_->getBodyJacobian(franka::Frame::kJoint4);
  std::array<double, 42> endeffector_jacobian_wrt_base =
      diana_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                       "-------------------------------------------------------------");
  RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "mass :" << mass);
  RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "coriolis :" << coriolis);
  RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "gravity :" << gravity);
  RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "joint_pose :" << pose);
  RCLCPP_INFO_STREAM_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "joint4_body_jacobian in joint4 frame :" << joint4_body_jacobian_wrt_joint4);
  RCLCPP_INFO_STREAM_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "end_effector_jacobian in base frame :" << endeffector_jacobian_wrt_base);
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                       "-------------------------------------------------------------");

  return controller_interface::return_type::OK;
}

}  // namespace diana_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(diana_example_controllers::ModelExampleController,
                       controller_interface::ControllerInterface)
