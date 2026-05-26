import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations["pkg_description"]
    init_height = context.launch_configurations["height"]
    debug_mode = context.launch_configurations["debug"]
    pkg_path = os.path.join(get_package_share_directory(package_description))

    xacro_file = os.path.join(pkg_path, "xacro", "robot.xacro")
    robot_description = xacro.process_file(
        xacro_file,
        mappings={"GAZEBO": "true", "CLASSIC": "true", "DEBUG": debug_mode},
    ).toxml()

    rviz_config_file = os.path.join(
        get_package_share_directory(package_description),
        "config",
        "visualize_urdf.rviz",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz_ocs2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(context.launch_configurations["rviz"]),
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"])]
        ),
        launch_arguments={
            "verbose": "false",
            "pause": context.launch_configurations["pause"],
        }.items(),
    )

    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        condition=IfCondition(context.launch_configurations["gui"]),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "robot",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            init_height,
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "publish_frequency": 20.0,
                "use_tf_static": True,
                "robot_description": robot_description,
                "ignore_timestamp": True,
            }
        ],
    )

    common_spawner_args = [
        "--controller-manager",
        "/controller_manager",
        "--controller-manager-timeout",
        "60.0",
        "--service-call-timeout",
        "60.0",
        "--switch-timeout",
        "60.0",
    ]

    leg_pd_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "leg_pd_controller",
            *common_spawner_args,
        ],
    )

    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            *common_spawner_args,
        ],
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            *common_spawner_args,
        ],
    )

    unitree_guide_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "unitree_guide_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60.0",
            "--service-call-timeout",
            "180.0",
            "--switch-timeout",
            "180.0",
        ],
    )

    unpause_gazebo = ExecuteProcess(
        cmd=["ros2", "service", "call", "/unpause_physics", "std_srvs/srv/Empty", "{}"],
        output="screen",
    )

    return [
        rviz,
        robot_state_publisher,
        gzserver,
        gzclient,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[unpause_gazebo, leg_pd_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=leg_pd_controller,
                on_exit=[joint_state_publisher],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_publisher,
                on_exit=[imu_sensor_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=imu_sensor_broadcaster,
                on_exit=[unitree_guide_controller],
            )
        ),
    ]


def generate_launch_description():
    pkg_description = DeclareLaunchArgument(
        "pkg_description",
        default_value="go1_description",
        description="package for robot description",
    )

    height = DeclareLaunchArgument(
        "height",
        default_value="0.43",
        description="Init height in simulation",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Whether to start rviz2",
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Whether to start gazebo client",
    )

    pause_arg = DeclareLaunchArgument(
        "pause",
        default_value="true",
        description="Whether to start gazebo paused",
    )

    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="false",
        description="Whether to fix the robot to world for suspended debugging",
    )

    return LaunchDescription(
        [
            pkg_description,
            height,
            rviz_arg,
            gui_arg,
            pause_arg,
            debug_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
