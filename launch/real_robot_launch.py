"""
This file is based on the sas_kuka_control_template
https://github.com/MarinhoLab/sas_kuka_control_template/blob/main/launch/real_robot_launch.py

Run this script in a different terminal window or tab. Be ready to close this, as this activates the real robot if the
connection is successful.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='sas_robot_driver_unitree_b1',
            executable='sas_robot_driver_unitree_b1_node',
            name='b1_1',
            namespace="sas_b1",
            parameters=[{
                "robot_name": "b1_1",
                "thread_sampling_time_sec": 0.002,
                "mode": "VelocityControl",
                "LIE_DOWN_ROBOT_WHEN_DEINITIALIZE": True,
                "sigterm_timeout": 30
            }]
        ),

    ])
