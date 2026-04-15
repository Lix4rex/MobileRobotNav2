import os

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

import launch_ros


packageName = "car"

rvizConfigRelativePath       = "config/config.rviz"
controllerParamsRelativePath = "config/controller_params.yaml"
robotControllerRelativePath  = "config/robot_controller.yaml"
slamParamsRelativePath       = "config/mapper_params_online_async.yaml"
nav2ParamsRelativePath       = "config/nav2_params.yaml"


def generate_launch_description():

    pkgPath = FindPackageShare(packageName).find(packageName)

    rvizConfigPath       = os.path.join(pkgPath, rvizConfigRelativePath)
    controllerParamsPath = os.path.join(pkgPath, controllerParamsRelativePath)
    robotControllerPath  = os.path.join(pkgPath, robotControllerRelativePath)
    slamParamsPath       = os.path.join(pkgPath, slamParamsRelativePath)
    nav2ParamsPath       = os.path.join(pkgPath, nav2ParamsRelativePath)

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare(packageName),
                "model",
                "real_robot.xacro"
            ])
        ]),
        value_type=str
    )

    return LaunchDescription([

        # ======================
        # ROBOT STATE PUBLISHER
        # ======================
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": False
            }]
        ),

        # ======================
        # RVIZ
        # ======================
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=["-d", rvizConfigPath],
            output='screen'
        ),

        # ======================
        # CONTROLLERS (ros2_control)
        # ======================

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster"],
                    parameters=[{'use_sim_time': False}],
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "omni_wheel_drive_controller",
                        "--param-file",
                        robotControllerPath
                    ],
                    parameters=[{'use_sim_time': False}],
                ),
            ],
        ),

        # ======================
        # TON NODE ROBOT (REAL CONTROL)
        # ======================
        Node(
            package="car",
            executable="car_controller",
            parameters=[
                controllerParamsPath,
                {'use_sim_time': False}
            ]
        ),

        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='LD19',
            output='screen',
            parameters=[
                {'product_name': 'LDLiDAR_LD19'},
                {'topic_name': 'scan'},
                {'frame_id': 'lidar_link'},
                {'port_name': '/dev/ttyUSB0'},
                {'port_baudrate': 230400},
                {'laser_scan_dir': True},
                {'enable_angle_crop_func': False},
                {'angle_crop_min': 135.0},
                {'angle_crop_max': 225.0}
            ]
        ),

        # ======================
        # SLAM TOOLBOX (REAL SENSOR INPUT)
        # ======================
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare("slam_toolbox"),
                            "launch",
                            "online_async_launch.py"
                        ])
                    ),
                    launch_arguments={
                        "use_sim_time": "false",
                        "slam_params_file": slamParamsPath
                    }.items(),
                )
            ]
        ),

        # ======================
        # NAV2 (REAL ROBOT)
        # ======================
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare("nav2_bringup"),
                            "launch",
                            "navigation_launch.py"
                        ])
                    ),
                    launch_arguments={
                        "use_sim_time": "false",
                        "params_file": nav2ParamsPath
                    }.items(),
                )
            ]
        ),
    ])