from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    threshold_arg = DeclareLaunchArgument(
        'threshold', default_value='0.5',
        description='Distance warning threshold in meters'
    )
    threshold = LaunchConfiguration('threshold')

    return LaunchDescription([
        threshold_arg,

        # ── BTVN_01 ────────────────────────────────────────────────────────
        Node(
            package='distance_warning',
            executable='distance_publisher',
            name='distance_publisher',
            output='screen',
        ),
        Node(
            package='distance_warning',
            executable='distance_listener',
            name='distance_listener',
            output='screen',
            parameters=[{'threshold': threshold}],
        ),
        Node(
            package='distance_warning',
            executable='set_threshold_service',
            name='set_threshold_service',
            output='screen',
            parameters=[{'threshold': threshold}],
        ),
        Node(
            package='distance_warning',
            executable='distance_action_server',
            name='distance_action_server',
            output='screen',
            parameters=[{'threshold': threshold}],
        ),

        # ── BTVN_02 ────────────────────────────────────────────────────────
        Node(
            package='distance_warning',
            executable='distance_tf_broadcaster',
            name='distance_tf_broadcaster',
            output='screen',
        ),
        Node(
            package='distance_warning',
            executable='distance_tf_listener',
            name='distance_tf_listener',
            output='screen',
            parameters=[{'tf_threshold': 1.0}],
        ),
        Node(
            package='distance_warning',
            executable='distance_publisher_qos',
            name='distance_publisher_qos',
            output='screen',
        ),
        Node(
            package='distance_warning',
            executable='distance_listener_qos',
            name='distance_listener_qos',
            output='screen',
            parameters=[{'threshold': threshold}],
        ),
    ])
