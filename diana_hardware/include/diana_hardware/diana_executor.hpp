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

#include <rclcpp/rclcpp.hpp>

namespace diana_hardware {

class DianaExecutor : public rclcpp::executors::MultiThreadedExecutor {
 public:
  // Create an instance and start the internal thread
  DianaExecutor();
  DianaExecutor(const DianaExecutor&) = delete;
  DianaExecutor(DianaExecutor&&) = delete;

  DianaExecutor& operator=(const DianaExecutor&) = delete;
  DianaExecutor& operator=(DianaExecutor&&) = delete;

  // Stops the internal executor and joins with the internal thread
  ~DianaExecutor() override;

 private:
  std::thread executor_spin_;

  // Executor thread starts spining the multithreadedExecutor
  void run();

  // Cancel any spinning ROS executor
  void shutdown();
};
}  // namespace diana_hardware
