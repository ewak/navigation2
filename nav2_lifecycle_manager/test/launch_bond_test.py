#! /usr/bin/env python3
# Copyright (c) 2020 Samsung Research America
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

import os
import sys

from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService


def generate_launch_description():
    tmux_gdb_prefix = (
        "tmux split-window "
        + get_package_prefix("nav2_bringup")
        + "/lib/nav2_bringup/gdb_tmux_splitwindow.sh"
    )
    logme = LogInfo(msg=f"tmux_gdb_prefix={tmux_gdb_prefix}")
    return LaunchDescription(
        [
            logme,
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_test',
                output='screen',
                prefix=tmux_gdb_prefix,
                parameters=[
                    {'use_sim_time': False},
                    {'autostart': False},
                    {'node_names': ['bond_tester']},
                ],
            ),
        ]
    )


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    testExecutable = os.getenv('TEST_EXECUTABLE') or "TEST_EXECUTABLE_not_specified"

    test1_action = ExecuteProcess(
        cmd=[testExecutable], name='test_bond_gtest', output='screen',
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
