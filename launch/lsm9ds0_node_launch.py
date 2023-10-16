from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="lsm9ds0_ros",
                executable="lsm9ds0_node",
                name="lsm9ds0_node",
                parameters=[
                    {"frame_id": "imu_lsm9ds0"},
                    {"pub_rate": 50},
                ],
            )
        ]
    )
