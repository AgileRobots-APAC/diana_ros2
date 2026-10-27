// Copyright (c) 2026 Agile Robots SE
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

#include <array>
#include <map>
#include <optional>
#include <string>

namespace diana_hardware {

/// Default impedance of the internal controller, used by the joint/Cartesian position and velocity
/// interfaces.
struct DefaultRobotBehavior {
  std::array<double, 7> joint_stiffness;
  std::array<double, 6> cartesian_stiffness;  // [x, y, z, roll, pitch, yaw]
};

/**
 * Returns the default behavior of the given Diana robot type.
 *
 * @param[in] robot_type the robot type, e.g. "diana7g2".
 * @return the default behavior, or std::nullopt if the robot type is unknown.
 */
inline std::optional<DefaultRobotBehavior> getDefaultRobotBehavior(const std::string& robot_type) {
  // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers)
  static const std::map<std::string, DefaultRobotBehavior> kDefaultRobotBehaviors{
      // Same as setDefaultBehavior() in libfranka/examples/<arm>/examples_common.cpp of the
      // diana_libfranka_overlay patch for each arm.
      {"diana3",
       {{600.0, 600.0, 400.0, 100.0, 100.0, 100.0, 100.0},
        {600.0, 600.0, 600.0, 200.0, 200.0, 200.0}}},
      {"diana7",
       {{2000.0, 2000.0, 2000.0, 1000.0, 700.0, 700.0, 700.0},
        {6000.0, 6000.0, 6000.0, 1000.0, 1000.0, 1000.0}}},
      {"diana7g2",
       {{2000.0, 2000.0, 2000.0, 1000.0, 700.0, 700.0, 700.0},
        {6000.0, 6000.0, 6000.0, 1000.0, 1000.0, 1000.0}}},
  };
  // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)

  auto it = kDefaultRobotBehaviors.find(robot_type);
  if (it == kDefaultRobotBehaviors.end()) {
    return std::nullopt;
  }
  return it->second;
}

}  // namespace diana_hardware
