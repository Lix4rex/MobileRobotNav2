from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package="car",
            executable="screen_monitor"
        ),

        Node(
            package="car",
            executable="match_monitor"
        ),

        Node(
            package="car",
            executable="calibration_monitor"
        ),

        Node(
            package="car",
            executable="oesophage_controller"
        ),

        Node(
            package="car",
            executable="stomach_controller"
        ),

        Node(
            package="car",
            executable="anus_controller"
        ),

    ])