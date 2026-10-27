#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/component_parser.hpp>

#include <diana_hardware_mocks/diana_active_control_mock.hpp>
#include <diana_hardware_mocks/diana_hardware_robot_mock.hpp>
#include <diana_hardware_mocks/diana_robot_mock.hpp>
#include "test_utils.hpp"

using namespace std::chrono_literals;

class DianaActionServerTests
    : public ::testing::TestWithParam<
          std::pair<std::function<void(std::shared_ptr<MockRobot> mock_robot)>,
                    rclcpp_action::ResultCode>> {
 public:
  auto SetUp() -> void override {
    auto urdf_string = readFileToString(TEST_CASE_DIRECTORY + robot_type + ".urdf");
    auto parsed_hardware_infos = hardware_interface::parse_control_resources_from_urdf(urdf_string);
    auto number_of_expected_hardware_components = 1;

    ASSERT_EQ(parsed_hardware_infos.size(), number_of_expected_hardware_components);

    default_hardware_info = parsed_hardware_infos[0];
    default_diana_hardware_interface.on_init(default_hardware_info);
  }

  auto TearDown() -> void override {
    default_diana_hardware_interface.on_deactivate(rclcpp_lifecycle::State());
  }

 protected:
  std::string robot_type{"diana3"};
  std::shared_ptr<MockRobot> default_mock_robot = std::make_shared<MockRobot>();
  std::shared_ptr<MockDianaRobot> default_mock_diana_robot = std::make_shared<MockDianaRobot>();
  hardware_interface::HardwareInfo default_hardware_info;
  diana_hardware::DianaHardwareInterface default_diana_hardware_interface{default_mock_robot,
                                                                             robot_type};
  /* Helper function to get the response of a action service */
  template <typename action_client_type>
  void get_action_service_response(
      const std::string& action_name,
      rclcpp_action::ResultCode result_code,
      const std::optional<std::function<void(std::shared_ptr<MockRobot> mock_robot)>>&
          mock_function = std::nullopt,
      const std::optional<typename action_client_type::Goal>& goal_msg_opt = std::nullopt) {
    if (mock_function.has_value()) {
      mock_function.value()(default_mock_robot);
    }

    auto node = rclcpp::Node::make_shared("test_node");

    auto client = rclcpp_action::create_client<action_client_type>(node, action_name);
    if (!client->wait_for_action_server(20s)) {
      ASSERT_TRUE(false) << "Action not available after waiting";
    }
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    bool is_finished = false;
    std::optional<rclcpp_action::ResultCode> received_code;
    auto goal_msg = typename action_client_type::Goal();
    if (goal_msg_opt.has_value()) {
      goal_msg = goal_msg_opt.value();
    }

    auto send_goal_options = typename rclcpp_action::Client<action_client_type>::SendGoalOptions();
    send_goal_options.goal_response_callback = [](const auto& future_result) {
      auto goal_handle = future_result.get();
      ASSERT_TRUE(goal_handle) << "Goal was rejected by the action server";
    };
    send_goal_options.feedback_callback = [](auto, auto) {};
    send_goal_options.result_callback = [&](const auto& result) {
      received_code = result.code;
      is_finished = true;
    };

    client->async_send_goal(goal_msg, send_goal_options);
    constexpr auto kActionTimeout = 5s;
    const auto end_point = std::chrono::system_clock::now() + kActionTimeout;
    while (!is_finished && std::chrono::system_clock::now() <= end_point) {
      executor.spin_some();
    }
    ASSERT_TRUE(is_finished) << "Timed out waiting for action result after " << kActionTimeout.count()
                             << "s";

    ASSERT_TRUE(received_code.has_value());
    ASSERT_EQ(received_code.value(), result_code);
  }
};

class DianaAutomaticRecoveryActionServerTests : public DianaActionServerTests {};

TEST_P(DianaAutomaticRecoveryActionServerTests,
       whenErrorRecoveryActionTriggered_thenErrorRecoveryServiceCallExecuted) {
  auto param = GetParam();

  get_action_service_response<diana_msgs::action::ErrorRecovery>("action_server/error_recovery",
                                                                  param.second, param.first);
}

INSTANTIATE_TEST_SUITE_P(
    DianaAutomaticRecoveryActionServerTestsInstantiation,
    DianaAutomaticRecoveryActionServerTests,
    ::testing::Values(std::make_pair(
                          [](std::shared_ptr<MockRobot> mock_robot) {
                            EXPECT_CALL(*mock_robot, automaticErrorRecovery()).Times(1);
                          },
                          rclcpp_action::ResultCode::SUCCEEDED),
                      std::make_pair(
                          [](std::shared_ptr<MockRobot> mock_robot) {
                            EXPECT_CALL(*mock_robot, automaticErrorRecovery())
                                .Times(1)
                                .WillRepeatedly(testing::Throw(franka::CommandException("")));
                          },
                          rclcpp_action::ResultCode::ABORTED),
                      std::make_pair(
                          [](std::shared_ptr<MockRobot> mock_robot) {
                            EXPECT_CALL(*mock_robot, automaticErrorRecovery())
                                .Times(1)
                                .WillRepeatedly(testing::Throw(franka::NetworkException("")));
                          },
                          rclcpp_action::ResultCode::ABORTED),
                      std::make_pair(
                          [](std::shared_ptr<MockRobot> mock_robot) {
                            EXPECT_CALL(*mock_robot, automaticErrorRecovery())
                                .Times(1)
                                .WillRepeatedly(testing::Throw(
                                    franka::InvalidOperationException("not in error state")));
                          },
                          rclcpp_action::ResultCode::ABORTED)));
