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

#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "test_move_to_start_example_controller.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

const double k_EPS = 1e-5;

const std::map<std::string, std::vector<double>> kExpectedStartJointConfigurations{
    {"diana3", {0.0, -M_PI_4, 0.0, 3.0 * M_PI_4, 0.0, M_PI_2, 0.0}},
    {"diana7", {0.0, -M_PI_4, 0.0, 3.0 * M_PI_4, 0.0, -M_PI_2, 0.0}},
    {"diana7g2", {0.0, -M_PI_4, 0.0, 3.0 * M_PI_4, 0.0, M_PI_2, 0.0}},
};

void MoveToStartExampleControllerTest::SetUpTestSuite() {
  rclcpp::init(0, nullptr);
}

void MoveToStartExampleControllerTest::TearDownTestSuite() {
  rclcpp::shutdown();
}

void MoveToStartExampleControllerTest::SetUp() {
  controller_ = std::make_unique<diana_example_controllers::MoveToStartExampleController>();
}

void MoveToStartExampleControllerTest::TearDown() {
  controller_.reset(nullptr);
}

void MoveToStartExampleControllerTest::SetUpController(const std::string& robot_type) {
  const auto result = controller_->init("test_move_to_start_example");
  ASSERT_EQ(result, controller_interface::return_type::OK);
  controller_->get_node()->set_parameter({"robot_type", robot_type});
  std::vector<LoanedCommandInterface> command_ifs;
  std::vector<LoanedStateInterface> state_ifs;

  command_ifs.emplace_back(joint_1_pos_cmd_);
  command_ifs.emplace_back(joint_2_pos_cmd_);
  command_ifs.emplace_back(joint_3_pos_cmd_);
  command_ifs.emplace_back(joint_4_pos_cmd_);
  command_ifs.emplace_back(joint_5_pos_cmd_);
  command_ifs.emplace_back(joint_6_pos_cmd_);
  command_ifs.emplace_back(joint_7_pos_cmd_);

  state_ifs.emplace_back(joint_1_pos_state_);
  state_ifs.emplace_back(joint_1_vel_state_);
  state_ifs.emplace_back(joint_2_pos_state_);
  state_ifs.emplace_back(joint_2_vel_state_);
  state_ifs.emplace_back(joint_3_pos_state_);
  state_ifs.emplace_back(joint_3_vel_state_);
  state_ifs.emplace_back(joint_4_pos_state_);
  state_ifs.emplace_back(joint_4_vel_state_);
  state_ifs.emplace_back(joint_5_pos_state_);
  state_ifs.emplace_back(joint_5_vel_state_);
  state_ifs.emplace_back(joint_6_pos_state_);
  state_ifs.emplace_back(joint_6_vel_state_);
  state_ifs.emplace_back(joint_7_pos_state_);
  state_ifs.emplace_back(joint_7_vel_state_);

  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
}

void MoveToStartExampleControllerTest::SetGains() {
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", D_gains_});
}

TEST_P(MoveToStartExampleControllerTest, controller_gains_not_set_failure) {
  SetUpController(GetParam());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

TEST_P(MoveToStartExampleControllerTest, controller_gain_empty) {
  SetUpController(GetParam());
  controller_->get_node()->set_parameter({"k_gains", std::vector<double>()});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

TEST_P(MoveToStartExampleControllerTest, controller_damping_gain_empty) {
  SetUpController(GetParam());
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", std::vector<double>()});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

TEST_P(MoveToStartExampleControllerTest, correct_controller_gains_success) {
  SetUpController(GetParam());
  SetGains();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_P(MoveToStartExampleControllerTest, correct_setup_on_activate_expect_success) {
  SetUpController(GetParam());
  SetGains();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_P(MoveToStartExampleControllerTest, correct_setup_on_update_expect_ok) {
  SetUpController(GetParam());
  SetGains();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto time = rclcpp::Time(0);
  auto duration = rclcpp::Duration(0, 0);

  ASSERT_EQ(controller_->update(time, duration), controller_interface::return_type::OK);

  EXPECT_NEAR(joint_1_pos_cmd_.get_value(), 0.0, k_EPS);
  EXPECT_NEAR(joint_2_pos_cmd_.get_value(), 0.0, k_EPS);
  EXPECT_NEAR(joint_3_pos_cmd_.get_value(), 0.0, k_EPS);
  EXPECT_NEAR(joint_4_pos_cmd_.get_value(), 0.0, k_EPS);
  EXPECT_NEAR(joint_5_pos_cmd_.get_value(), 0.0, k_EPS);
  EXPECT_NEAR(joint_6_pos_cmd_.get_value(), 0.0, k_EPS);
  EXPECT_NEAR(joint_7_pos_cmd_.get_value(), 0.0, k_EPS);
}

TEST_P(MoveToStartExampleControllerTest, default_start_joint_configuration_matches_robot_type) {
  const auto start_joint_configuration =
      diana_example_controllers::MoveToStartExampleController::defaultStartJointConfiguration(
          GetParam());
  ASSERT_TRUE(start_joint_configuration.has_value());

  const auto& expected = kExpectedStartJointConfigurations.at(GetParam());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR((*start_joint_configuration)(static_cast<Eigen::Index>(i)), expected[i], k_EPS)
        << "joint" << i + 1;
  }
}

TEST_P(MoveToStartExampleControllerTest, start_joint_configuration_override_success) {
  SetUpController(GetParam());
  SetGains();
  controller_->get_node()->set_parameter(
      {"start_joint_configuration", std::vector<double>{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_P(MoveToStartExampleControllerTest, start_joint_configuration_wrong_size_failure) {
  SetUpController(GetParam());
  SetGains();
  controller_->get_node()->set_parameter(
      {"start_joint_configuration", std::vector<double>{0.0, 0.0, 0.0}});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

INSTANTIATE_TEST_SUITE_P(DianaRobots,
                         MoveToStartExampleControllerTest,
                         ::testing::Values("diana3", "diana7", "diana7g2"),
                         [](const ::testing::TestParamInfo<std::string>& info) {
                           return info.param;
                         });

TEST_F(MoveToStartExampleControllerRobotTypeTest, empty_robot_type_failure) {
  SetUpController("");
  SetGains();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

TEST_F(MoveToStartExampleControllerRobotTypeTest, unsupported_robot_type_failure) {
  SetUpController("fr3");
  SetGains();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

TEST_F(MoveToStartExampleControllerRobotTypeTest,
       unsupported_robot_type_has_no_default_start_joint_configuration) {
  EXPECT_FALSE(
      diana_example_controllers::MoveToStartExampleController::defaultStartJointConfiguration("fr3")
          .has_value());
  EXPECT_FALSE(
      diana_example_controllers::MoveToStartExampleController::defaultStartJointConfiguration("")
          .has_value());
}
