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

#include "diana_semantic_components/diana_robot_model.hpp"

#include <cstring>
#include <iostream>
#include "rclcpp/logging.hpp"
namespace {

// Example implementation of bit_cast: https://en.cppreference.com/w/cpp/numeric/bit_cast
template <class To, class From>
std::enable_if_t<sizeof(To) == sizeof(From) && std::is_trivially_copyable<From>::value &&
                     std::is_trivially_copyable<To>::value,
                 To>
bit_cast(const From& src) noexcept {
  static_assert(std::is_trivially_constructible<To>::value,
                "This implementation additionally requires "
                "destination type to be trivially constructible");

  To dst;
  std::memcpy(&dst, &src, sizeof(To));
  return dst;
}

}  // namespace

namespace diana_semantic_components {

DianaRobotModel::DianaRobotModel(const std::string& diana_model_interface_name,
                                 const std::string& diana_state_interface_name)
    : SemanticComponentInterface(diana_model_interface_name, 2) {
  diana_model_interface_name_ = diana_model_interface_name;
  diana_state_interface_name_ = diana_state_interface_name;
  interface_names_.emplace_back(diana_model_interface_name);
  interface_names_.emplace_back(diana_state_interface_name);
}

void DianaRobotModel::initialize() {
  auto diana_state_interface =
      std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](const auto& interface) {
        return interface.get().get_name() == diana_state_interface_name_;
      });

  auto diana_model_interface =
      std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](const auto& interface) {
        return interface.get().get_name() == diana_model_interface_name_;
      });

  if (diana_state_interface != state_interfaces_.end() &&
      diana_model_interface != state_interfaces_.end()) {
    robot_model_ = bit_cast<diana_hardware::Model*>((*diana_model_interface).get().get_value());
    robot_state_ = bit_cast<franka::RobotState*>((*diana_state_interface).get().get_value());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("diana_model_semantic_component"),
                 "Diana interface does not exist! Did you assign the loaned state in the "
                 "controller?");
    throw std::runtime_error("Diana state interfaces does not exist");
  }
  initialized_ = true;
}
}  // namespace diana_semantic_components
