#pragma once

#include <gmock/gmock.h>

#include <string>

#include <diana_hardware/robot.hpp>

#include "diana_model_mock.hpp"

class MockRobot : public diana_hardware::Robot {
 public:
  MockRobot() {
    ON_CALL(*this, getRobotModel()).WillByDefault(testing::Return(kMockRobotModelUrdf));
    ON_CALL(*this, getRobotType()).WillByDefault([this]() { return Robot::getRobotType(); });
  }

  MOCK_METHOD(void, initializeJointPositionInterface, (), (override));
  MOCK_METHOD(void, initializeCartesianVelocityInterface, (), (override));
  MOCK_METHOD(void, initializeCartesianPoseInterface, (), (override));
  MOCK_METHOD(void, initializeTorqueInterface, (), (override));
  MOCK_METHOD(void, initializeJointVelocityInterface, (), (override));
  MOCK_METHOD(void, stopRobot, (), (override));
  MOCK_METHOD(franka::RobotState, readOnce, (), (override));
  MOCK_METHOD(MockModel*, getModel, (), (override));
  MOCK_METHOD(void, writeOnce, ((const std::vector<double>&)), (override));
  MOCK_METHOD(void,
              writeOnce,
              ((const std::vector<double>&), (const std::vector<double>&)),
              (override));
  MOCK_METHOD(void,
              setJointStiffness,
              (const diana_msgs::srv::SetJointStiffness::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setCartesianStiffness,
              (const diana_msgs::srv::SetCartesianStiffness::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void, setLoad, (const diana_msgs::srv::SetLoad::Request::SharedPtr&), (override));
  MOCK_METHOD(void,
              setTCPFrame,
              (const diana_msgs::srv::SetTCPFrame::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setStiffnessFrame,
              (const diana_msgs::srv::SetStiffnessFrame::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setForceTorqueCollisionBehavior,
              (const diana_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setFullCollisionBehavior,
              (const diana_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void, automaticErrorRecovery, (), (override));
  MOCK_METHOD(std::shared_ptr<franka::Robot>, getRobot, (), (override));
  MOCK_METHOD(std::string, getRobotModel, (), (override));
  MOCK_METHOD(std::string, getRobotType, (), (override));
  MOCK_METHOD(const franka::RobotState&, getCurrentState, (), (override));

 private:
  static constexpr const char* kMockRobotModelUrdf = "<robot name=\"diana3\"/>";
};
