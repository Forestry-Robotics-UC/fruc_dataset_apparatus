# Copyright 2024 Forestry Robotics Lab. All Rights Reserved.
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
Image transport relay launch file for realsense camera.
Creates compressed JPEG (lossy) and PNG (lossless) versions of RGB image.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    camera_namespace_arg = DeclareLaunchArgument(
        'camera_namespace',
        default_value='camera',
        description='namespace for camera'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='color',
        description='name of the color image topic'
    )

    # Compressed relay node (JPEG - lossy, high quality)
    compressed_relay = Node(
        package='image_transport',
        executable='republish',
        name='color_image_compressed_relay',
        namespace=LaunchConfiguration('camera_namespace'),
        remappings=[
            ('in', [LaunchConfiguration('camera_namespace'), '/', LaunchConfiguration('camera_name'), '/image_raw']),
            ('out', [LaunchConfiguration('camera_namespace'), '/', LaunchConfiguration('camera_name'), '/image_raw/compressed']),
        ],
        arguments=['raw', 'compressed', '--ros-args', '-p', 'use_sim_time:=false', '-p', 'jpeg_quality:=95'],
    )

    return LaunchDescription([
        camera_namespace_arg,
        camera_name_arg,
        compressed_relay,
    ])
