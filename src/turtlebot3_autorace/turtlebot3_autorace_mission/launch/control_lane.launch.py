#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
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
# Author: Hyungyu Kim

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    control_node = Node(
            package='turtlebot3_autorace_mission',
            executable='control_lane',
            name='control_lane',
            output='screen',
            remappings=[
                ('/control/lane', '/detect/lane'),
                ('/control/cmd_vel', '/cmd_vel')
            ]
        )
    
    traffic_light_param_path = os.path.join(
        get_package_share_directory('turtlebot3_autorace_detect'),
        'param',
        'traffic_light',
        'traffic_light.yaml'
    )
    
    crosswalk_stopper = Node(
            package='turtlebot3_autorace_mission',
            executable='crosswalk_stopper',
            name='crosswalk_stopper'
        )
    
    detect_traffic_light = Node(
        package='turtlebot3_autorace_detect',
        executable='detect_traffic_light',
        name='detect_traffic_light',
        output='screen',
        remappings=[
            # ('/detect/image_input', '/camera/image_raw'),
            ('/detect/image_input/compressed', '/camera/image_raw/compressed'),
            ('/detect/image_output', '/detect/image_traffic_light'),
            ('/detect/image_output/compressed', '/detect/image_traffic_light/compressed'),
            ('/detect/image_output_sub1', '/detect/image_red_light'),
            ('/detect/image_output_sub1/compressed', '/detect/image_red_light/compressed'),
            ('/detect/image_output_sub2', '/detect/image_yellow_light'),
            ('/detect/image_output_sub2/compressed', '/detect/image_yellow_light/compressed'),
            ('/detect/image_output_sub3', '/detect/image_green_light'),
            ('/detect/image_output_sub3/compressed', '/detect/image_green_light/compressed'),
            ],
        parameters=[
            traffic_light_param_path,
            {'is_calibration_mode': True}
            ]
        )
    
    return LaunchDescription([
        control_node,
        crosswalk_stopper,
        detect_traffic_light
    ])
