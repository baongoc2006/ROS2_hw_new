from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    threshold_arg = DeclareLaunchArgument(
        'threshold',
        default_value='0.5',
        description='Distance warning threshold in meters'
    )
    threshold = LaunchConfiguration('threshold')

    return LaunchDescription([
        threshold_arg,

        # Publisher: phát khoảng cách ngẫu nhiên @ 1 Hz
        Node(
            package='distance_warning',
            executable='distance_publisher',
            name='distance_publisher',
            output='screen',
        ),

        # Listener: subscribe và cảnh báo khi vượt ngưỡng
        Node(
            package='distance_warning',
            executable='distance_listener',
            name='distance_listener',
            output='screen',
            parameters=[{'threshold': threshold}],
        ),

        # Service server: thay đổi threshold runtime
        Node(
            package='distance_warning',
            executable='set_threshold_service',
            name='set_threshold_service',
            output='screen',
            parameters=[{'threshold': threshold}],
        ),

        # Action server: kiểm tra khoảng cách theo yêu cầu
        Node(
            package='distance_warning',
            executable='distance_action_server',
            name='distance_action_server',
            output='screen',
            parameters=[{'threshold': threshold}],
        ),
    ])
