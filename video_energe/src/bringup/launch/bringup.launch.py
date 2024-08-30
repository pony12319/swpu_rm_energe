import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from setuptools import setup
from glob import glob
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='video_test',
            executable='video_test_node',
            name='video_test',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='img_pro',
            executable='img_pro_node',
            name='img_pro_node_test',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='para_cal',
            executable='para_cal_node',
            name='para_cal_node_test',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='pitch_yaw_solve',
            executable='pitch_yaw_solve_node',
            name='pitch_yaw_solve_test',
            output='screen',
            emulate_tty=True,
        ),
    ])