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
robotControllerRelativePath  = "config/localization_robot_controller.yaml"
nav2ParamsRelativePath       = "config/nav2_params.yaml"
ekfConfigRelativePath        = "config/ekf.yaml"

def generate_launch_description():

    pkgPath              = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)
    world_path           = os.path.join(pkgPath, worldRelativePath)
    rvizConfigPath       = os.path.join(pkgPath, rvizConfigRelativePath)
    controllerParamsPath = os.path.join(pkgPath, controllerParamsRelativePath)
    robotControllerPath  = os.path.join(pkgPath, robotControllerRelativePath)
    ekfConfigPath        = os.path.join(pkgPath, ekfConfigRelativePath)
    nav2ParamsPath       = os.path.join(pkgPath, nav2ParamsRelativePath)
    

    
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare(packageName), 
                "model",
                "localization-robot",
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

    return LaunchDescription([

        gazebo,

        # Bridges

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

        # Imu
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
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
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    output="both",
                    parameters=[{
                        "robot_description":robot_description,
                        "use_sim_time": True
                    }]
                ),
            ]
        ),

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
        ),

        Node(
            package="car",
            executable="gt_node"
        ),

        # EKF
        TimerAction(
            period=3.5,
            actions=[
                Node(
                    package="robot_localization",
                    executable="ekf_node",
                    name="ekf_filter_node",
                    output="screen",
                    parameters=[ekfConfigPath, {'use_sim_time': True}],
                ),
            ]
        ),


        # NAV2
        # TimerAction(
        #     period=5.0,
        #     actions=[
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource(
        #                 PathJoinSubstitution([
        #                     FindPackageShare("nav2_bringup"),
        #                     "launch",
        #                     "navigation_launch.py"
        #                 ])
        #             ),
        #             launch_arguments={
        #                 "use_sim_time": "true",
        #                 "params_file": nav2ParamsPath
        #             }.items(),
        #         ),
        #     ]
        # ),

    ])