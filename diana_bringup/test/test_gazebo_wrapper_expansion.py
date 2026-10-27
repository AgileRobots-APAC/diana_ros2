# Copyright (c) 2025 Franka Robotics GmbH
# Copyright (c) 2026 Agile Robots SE (modifications: adapted for Diana robots)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Expansion guard for all Gazebo bringup wrappers.

Ensures that each .gazebo.urdf.xacro expands without XacroException and
produces at least one <ros2_control> block (hardware not lost after migration).
"""

import pytest

from ros2_control_test_helpers import (
    expand_wrapper_urdf,
    get_gazebo_bringup_wrapper,
)

GAZEBO_WRAPPERS = [
    ('diana_arm.gazebo.xacro', {'robot_type': 'diana3'}),
    ('diana_arm.gazebo.xacro', {'robot_type': 'diana7'}),
    ('diana_arm.gazebo.xacro', {'robot_type': 'diana7g2'}),
]


@pytest.mark.parametrize(
    'wrapper_name,mappings',
    GAZEBO_WRAPPERS,
    ids=[w[0] for w in GAZEBO_WRAPPERS],
)
class TestGazeboWrapperExpansion:
    """Each gazebo wrapper must expand cleanly and emit ros2_control hardware."""

    def test_expands_without_exception(self, wrapper_name: str, mappings: dict):
        """xacro.process_file succeeds (no XacroException on stale params)."""
        wrapper = get_gazebo_bringup_wrapper(wrapper_name)
        urdf = expand_wrapper_urdf(wrapper, mappings=mappings)
        assert len(urdf) > 0

    def test_contains_ros2_control_block(self, wrapper_name: str, mappings: dict):
        """Expanded URDF contains at least one <ros2_control> element."""
        wrapper = get_gazebo_bringup_wrapper(wrapper_name)
        urdf = expand_wrapper_urdf(wrapper, mappings=mappings)
        assert '<ros2_control' in urdf, (
            f'{wrapper_name} expanded but contains no <ros2_control> block — '
            f'hardware emission lost'
        )
