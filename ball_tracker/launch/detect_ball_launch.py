from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    color = LaunchConfiguration('color')

    param_file = PathJoinSubstitution([
        FindPackageShare('ball_tracker'),
        'config',
        PythonExpression(["'", color, "_ball.yaml'"]) 
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'color',
            default_value='red',
            description='Color of the ball to track (red, green, blue)'
        ),

        Node(
            package='ball_tracker',
            executable='detect_ball',
            name='detect_ball',
            output='screen',
            parameters=[param_file],
            remappings=[
                ('/image_in', '/camera/image_raw')
            ]
        )
    ])
