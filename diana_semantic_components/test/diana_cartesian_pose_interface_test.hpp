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

#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

#include "diana_semantic_components/diana_cartesian_pose_interface.hpp"
#include "gmock/gmock.h"

class DianaCartesianPoseTestFriend : public diana_semantic_components::DianaCartesianPoseInterface {
 public:
  explicit DianaCartesianPoseTestFriend(const bool elbow_activate)
      : diana_semantic_components::DianaCartesianPoseInterface(elbow_activate) {}

  virtual ~DianaCartesianPoseTestFriend() = default;
};

class DianaCartesianPoseTest : public ::testing::Test {
 public:
  void constructDianaCartesianPoseInterface(bool elbow_active);
  void TearDown();

 private:
  void setUpHWCommandInterfaces(bool elbow_active);
  void setUpHWStateInterfaces(bool elbow_active);

 protected:
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>>
      pose_command_interfaces_container, elbow_command_interfaces_container;
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> pose_state_interfaces_container,
      elbow_state_interfaces_container;
  std::array<std::string, 2> hw_elbow_command_names_{"joint_3_position", "joint_4_sign"};

  std::array<double, 16> hw_cartesian_pose_command_{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0,
                                                    0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
  std::array<double, 2> hw_elbow_command_{0.0, 0.0};

  std::array<double, 16> cartesian_pose_state_{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                               0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  std::array<double, 2> elbow_state_{1.0, 1.0};

  std::unique_ptr<DianaCartesianPoseTestFriend> diana_cartesian_command_friend;
  std::vector<hardware_interface::LoanedCommandInterface> temp_command_interfaces;
  std::vector<hardware_interface::LoanedStateInterface> temp_state_interfaces;

  const std::string cartesian_pose_command_interface_name_{"cartesian_pose_command"};
  const std::string elbow_command_interface_name_{"elbow_command"};
  const std::string cartesian_pose_state_interface_name_{"cartesian_pose_state"};
  const std::string elbow_state_interface_name_{"elbow_state"};
};
