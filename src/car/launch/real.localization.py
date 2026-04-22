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
controllerParamsRelativePath = "config/real/controller_params.yaml"
robotControllerRelativePath  = "config/real/robot_controller.yaml"
nav2ParamsRelativePath       = "config/real/nav2_params.yaml"
ekfConfigRelativePath        = "config/ekf.yaml"
mapFileRelativePath          = "config/map/big_empty_map.yaml"


def generate_launch_description():

    pkgPath = FindPackageShare(packageName).find(packageName)

    rvizConfigPath       = os.path.join(pkgPath, rvizConfigRelativePath)
    controllerParamsPath = os.path.join(pkgPath, controllerParamsRelativePath)
    robotControllerPath  = os.path.join(pkgPath, robotControllerRelativePath)
    nav2ParamsPath       = os.path.join(pkgPath, nav2ParamsRelativePath)
    ekfConfigPath        = os.path.join(pkgPath, ekfConfigRelativePath)
    mapFilePath          = os.path.join(pkgPath, mapFileRelativePath)    

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare(packageName),
                "model",
                "real-robot",
                "robot.xacro"
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

        Node( 
            package='tf2_ros', 
            executable='static_transform_publisher', 
            arguments=[ '0', '0', '0', '0', '0', '0', 'map', 'odom' ], 
            parameters=[{'use_sim_time': False}], 
        ),

        # Node(
        #     package='topic_tools',
        #     executable='relay',
        #     arguments=[
        #         '/micro_controller/joint_states',
        #         '/joint_states'
        #     ],
        #     output='screen'
        # ),

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
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robotControllerPath],
            output="screen"
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

        # Node(
        #     package='ldlidar_stl_ros2',
        #     executable='ldlidar_stl_ros2_node',
        #     name='LD19',
        #     output='screen',
        #     parameters=[
        #         {'product_name': 'LDLiDAR_LD19'},
        #         {'topic_name': 'scan'},
        #         {'frame_id': 'lidar_link'},
        #         {'port_name': '/dev/ttyUSB0'},
        #         {'port_baudrate': 230400},
        #         {'laser_scan_dir': True},
        #         {'enable_angle_crop_func': False},
        #         {'angle_crop_min': 135.0},
        #         {'angle_crop_max': 225.0}
        #     ]
        # ),



        # EKF
        # TimerAction(
        #     period=3.5,
        #     actions=[
        #         Node(
        #             package="robot_localization",
        #             executable="ekf_node",
        #             name="ekf_filter_node",
        #             output="screen",
        #             parameters=[ekfConfigPath, {'use_sim_time': True}],
        #         ),
        #     ]
        # ),

        # NAV2
        TimerAction(
            period=7.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare("nav2_bringup"),
                            "launch",
                            "bringup_launch.py"
                        ])
                    ),
                    launch_arguments={
                        "slam": "False",
                        "map": mapFilePath,
                        "use_sim_time": "false",
                        "params_file": nav2ParamsPath,
                        "autostart": "true"
                    }.items(),
                ),
            ]
        ),
    ])