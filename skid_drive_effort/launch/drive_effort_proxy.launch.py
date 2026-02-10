from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    cfg = PathJoinSubstitution([FindPackageShare("skid_drive_effort"), "config", "example.yaml"])
    return LaunchDescription([
        Node(
            package="skid_drive_effort",
            executable="drive_effort_proxy",
            name="drive_effort_proxy",
            output="screen",
            parameters=[cfg],
        )
    ])
