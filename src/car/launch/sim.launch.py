import os

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

import launch_ros

from launch.actions import TimerAction


packageName = "car"
worldRelativePath            = "config/world.sdf"
rvizConfigRelativePath       = "config/config.rviz"
controllerParamsRelativePath = "config/controller_params.yaml"
robotControllerRelativePath  = "config/robot_controller.yaml"

def generate_launch_description():

    pkgPath              = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)
    world_path           = os.path.join(pkgPath, worldRelativePath)
    rvizConfigPath       = os.path.join(pkgPath, rvizConfigRelativePath)
    controllerParamsPath = os.path.join(pkgPath, controllerParamsRelativePath)
    robotControllerPath  = os.path.join(pkgPath, robotControllerRelativePath)

    
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare(packageName), 
                "model",
                "robot.xacro"
            ])
        ]),
        value_type=str
    )

    # --- GAZEBO (ton style) ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py"
                ])
            ]
        ),
        launch_arguments={
            "gz_args": [
                "-r -v 4 --physics-engine gz-physics-dartsim-plugin ",
                world_path
            ]
        }.items(),
    )


    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{
            "robot_description":robot_description,
            "use_sim_time": True
        }]
    )

    delayed_rsp = launch.actions.TimerAction(
        period=2.0,
        actions=[robot_state_publisher_node]
    )

    return LaunchDescription([

        gazebo,

        # Bridge

        #Clock
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # Lidar
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ]
        ),


        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'simple_robot',
                '-topic', 'robot_description'
            ],
            parameters=[{
                'use_sim_time': True
            }],
            output='screen'
        ),

        # robot_state_publisher
        delayed_rsp,

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[{'use_sim_time': True}],
            arguments=["-d", rvizConfigPath],
            output='screen'
        ),


        # Controllers

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster"],
                    parameters=[{'use_sim_time': True}],
                ),
                # Node(
                #     package="controller_manager",
                #     executable="spawner",
                #     arguments=["diff_drive_controller", "--param-file", robotControllerPath],
                #     parameters=[{'use_sim_time': True}],
                # ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["omni_wheel_drive_controller", "--param-file", robotControllerPath],
                    parameters=[{'use_sim_time': True}],
                ),
            ],
        ),

        Node(
            package="car",
            executable="car_controller",
            parameters = [controllerParamsPath, {'use_sim_time': True}]
        )
    ])