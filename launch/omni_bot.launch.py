from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Find the installed URDF file
    urdf_file = os.path.join(
        get_package_share_directory('gz_rosa_control'),
        'urdf',
        'omni_bot.urdf'
    )

    return LaunchDescription([
        # Start Gazebo with an empty world and the ROS factory plugin
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Spawn the robot in Gazebo using the installed URDF path
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf_file, '-entity', 'omni_bot'],
            output='screen'
        ),
        # Robot State Publisher with the installed URDF path
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_file],
            output='screen'
        ),
        # Joint State Publisher GUI (optional)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
    ])

