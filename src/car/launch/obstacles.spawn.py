import os

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


package_name = "car"
obstacle_relative_path = "model/obstacles.sdf"


def generate_launch_description():

    pkg_path = FindPackageShare(package_name).find(package_name)
    obstacle_path = os.path.join(pkg_path, obstacle_relative_path)

    def spawn_obstacle(name, x, y, z):
        return Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', name,
                '-file', obstacle_path,
                '-x', str(x),
                '-y', str(y),
                '-z', str(z)
            ],
            output='screen'
        )

    return LaunchDescription([
        spawn_obstacle('obstacle_1', 2.5, 0, 5),
        spawn_obstacle('obstacle_2', -3, 1, 5),
        spawn_obstacle('obstacle_3', 0, -2, 5),
    ])
