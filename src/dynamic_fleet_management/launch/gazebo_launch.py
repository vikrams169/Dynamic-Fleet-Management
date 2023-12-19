#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


for arg in sys.argv:
    if arg.startswith("node_count:="):
        count = int(arg.split(":=")[1])
    else:
        count = 10


def generate_launch_description():

    launch_file_dir = os.path.join(
        get_package_share_directory('dynamic_fleet_management'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(
        get_package_share_directory('dynamic_fleet_management'),
        'worlds',
        'empty_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    runner_node = ExecuteProcess(
        cmd=[[
            'ros2 run dynamic_fleet_management roomba_algo '+str(count)
        ]],
        shell=True
    )

    # Get the urdf file
    TURTLEBOT3_MODEL = 'waffle_pi'
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('dynamic_fleet_management'),
        'models',
        model_folder,
        'model.sdf'
    )
    ld = LaunchDescription()
    for i in range(count):
        robot_name = "robot_"+str(i)
        x_val = str(float(i-(count/2)))
        node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot_name,
                '-file', urdf_path,
                '-x', x_val,
                '-y', y_pose,
                '-z', '0.01',
                '-robot_namespace', robot_name
            ],
            output='screen',
        )
        ld.add_action(TimerAction(period=10.0+float(i*2),
                                  actions=[node],))

    # ld.add_action(TimerAction(period=20.0+float(count*2),
    #         actions=[runner_node],))

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)

    # ld.add_action(spawn_turtlebot_cmd)
    # ld.add_action(node_arg)

    return ld
