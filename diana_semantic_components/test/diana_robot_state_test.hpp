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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include "diana_semantic_components/diana_robot_state.hpp"
#include "franka/robot_state.h"
#include "gmock/gmock.h"

// implementing and friending so we can access member variables
class DianaRobotStateTestFriend : public diana_semantic_components::DianaRobotState {
  FRIEND_TEST(DianaRobotStateTest, validate_state_names_and_size);
  FRIEND_TEST(DianaRobotStateTest,
              givenDianaSemanticStateInitialized_whenMessageReturnedExpectsCorrectValues);
  FRIEND_TEST(DianaRobotStateTest, robot_state_ptr_uncasted_correctly);

 public:
  // Use generation of interface names
  explicit DianaRobotStateTestFriend(const std::string& name)
      : diana_semantic_components::DianaRobotState(
            name,
            get_robot_description("diana_semantic_components")) {}

  virtual ~DianaRobotStateTestFriend() = default;

 private:
  static std::string get_robot_description(const std::string& package_name) {
    std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
    std::string file_path = package_path + "/robot_description_test.txt";

    std::ifstream file(file_path);
    if (!file) {
      throw std::runtime_error("Failed to open file: " + file_path);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();

    return buffer.str();
  }
};

class DianaRobotStateTest : public ::testing::Test {
 public:
  void SetUp();

  void TearDown();

 protected:
  const size_t size = 1;
  const std::string robot_name = "diana3";
  const std::string diana_state_interface_name = "robot_state";
  franka::RobotState robot_state;
  franka::RobotState* robot_state_address = &robot_state;

  std::array<double, 7> joint_angles = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<double, 7> joint_velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 16> end_effector_pose = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,   0.0,
                                              0.0, 0.0, 1.0, 0.0, 2.2, 3.8, 93.23, 1.0};
  franka::RobotMode robot_mode = franka::RobotMode::kUserStopped;
  diana_msgs::msg::DianaRobotState diana_robot_state_msg;

  std::unique_ptr<DianaRobotStateTestFriend> diana_state_friend;

  std::vector<std::string> full_interface_names;
};
