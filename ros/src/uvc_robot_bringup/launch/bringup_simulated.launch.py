import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'uvc_robot_bringup'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('uvc_robot_gazebo'), 'launch', 'launch_sim.launch.py'
        )])
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )

    imu_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster"]
    )

    uvc_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["uvc_controller"]
    )

    return LaunchDescription([
        gazebo,
        TimerAction(period=5.0, actions=[joint_broad_spawner]),
        TimerAction(period=5.0, actions=[imu_broad_spawner]),
        TimerAction(period=5.0, actions=[uvc_controller_spawner])
    ])