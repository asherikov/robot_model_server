# Copyright (c) 2024 Open Source Robotics Foundation, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import unittest

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts


def generate_test_description():
    test_exe_arg = launch.actions.DeclareLaunchArgument(
        'test_exe',
        description='Path to executable test',
    )

    process_under_test = launch.actions.ExecuteProcess(
        cmd=[launch.substitutions.LaunchConfiguration('test_exe'),
             '--ros-args', '-r', '__ns:=/test_inertial',
             '-r', '/tf:=/test_inertial/tf',
             '-r', '/tf_static:=/test_inertial/tf_static'],
        output='screen',
    )

    urdf_file = os.path.join(os.path.dirname(__file__), 'inertial.urdf')
    with open(urdf_file, 'r', encoding='utf-8') as infp:
        robot_desc = infp.read()

    params = {
        'model.description': robot_desc,
        'visualization.inertia': True,
        'inertia.cumulative': True,
    }
    node_robot_model_server = Node(
        package='robot_model_server_ros',
        executable='robot_model_server_ros',
        namespace='test_inertial',
        output='screen',
        parameters=[params],
        remappings=[
            ('/tf', '/test_inertial/tf'),
            ('/tf_static', '/test_inertial/tf_static'),
        ]
    )

    return LaunchDescription([
        node_robot_model_server,
        test_exe_arg,
        process_under_test,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), locals()


class TestInertial(unittest.TestCase):

    def test_termination(self, process_under_test, proc_info):
        proc_info.assertWaitForShutdown(process=process_under_test, timeout=20)


@launch_testing.post_shutdown_test()
class InertialTestAfterShutdown(unittest.TestCase):

    def test_exit_code(self, process_under_test, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            process_under_test
        )

