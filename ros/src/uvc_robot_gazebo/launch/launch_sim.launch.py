import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_ros.descriptions


from launch_ros.actions import Node

def generate_launch_description():

    package_name='uvc_robot_gazebo'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('uvc_robot_description'), 'launch', 'rsp.launch.py'
        )])
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", 'empty.sdf']}.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "robot",
            "-topic",
            "/robot_description"
        ]
    )

    return LaunchDescription([
        rsp,
        gz_sim,
        spawn_entity
    ])