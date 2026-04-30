from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package="car",
            executable="oesophage_action_server"
        ),

        Node(
            package="car",
            executable="stomach_action_server"
        ),

        Node(
            package="car",
            executable="anus_action_server"
        ),

    ])