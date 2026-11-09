# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, Shutdown, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    robot_ip_parameter_name = "robot_ip"
    use_fake_hardware_parameter_name = "use_fake_hardware"
    fake_sensor_commands_parameter_name = "fake_sensor_commands"
    namespace_parameter_name = "namespace"
    load_gripper_parameter_name = "load_gripper"
    ee_id_parameter_name = "ee_id"
    planning_mode_parameter_name = "planning_mode"
    planner_id_parameter_name = "planner_id"
    planning_pipeline_parameter_name = "planning_pipeline"
    lock_orientation_parameter_name = "lock_orientation"

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    namespace = LaunchConfiguration(namespace_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    ee_id = LaunchConfiguration(ee_id_parameter_name)
    planning_mode = LaunchConfiguration(planning_mode_parameter_name)
    planner_id = LaunchConfiguration(planner_id_parameter_name)
    planning_pipeline = LaunchConfiguration(planning_pipeline_parameter_name)
    lock_orientation = LaunchConfiguration(lock_orientation_parameter_name)

    db_arg = DeclareLaunchArgument("db", default_value="False", description="Database flag")

    franka_xacro_file = os.path.join(
        get_package_share_directory("franka_bringup"),
        "urdf",
        "franka_arm.urdf.xacro",
    )

    robot_description_config = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            franka_xacro_file,
            " hand:=",
            load_gripper,
            " robot_ip:=",
            robot_ip,
            " ee_id:=",
            ee_id,
            " use_fake_hardware:=",
            use_fake_hardware,
            " fake_sensor_commands:=",
            fake_sensor_commands,
            " robot_type:=fr3",
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_config, value_type=str)}

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "fr3",
        "fr3.srdf.xacro",
    )

    robot_description_semantic_config = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            franka_semantic_xacro_file,
            " hand:=",
            load_gripper,
            " ee_id:=",
            ee_id,
        ]
    )

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_config, value_type=str)
    }

    kinematics_yaml = load_yaml("franka_fr3_moveit_config", "config/kinematics.yaml")

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/ResolveConstraintFrames "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("franka_fr3_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        "franka_fr3_moveit_config", "config/fr3_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="both",
        parameters=[robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("franka_moveit_compliant_control"),
        "config",
        "compliant_ros_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[robot_description, ros2_controllers_path],
        remappings=[("joint_states", "franka/joint_states")],
        output={"stdout": "screen", "stderr": "screen"},
        on_exit=Shutdown(),
    )

    load_controllers = []
    load_controllers.append(
        ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "controller_manager",
                "spawner",
                "fr3_arm_controller",
                "--inactive",
                "--controller-manager-timeout",
                "60",
                "--controller-manager",
                PathJoinSubstitution([namespace, "controller_manager"]),
            ],
            output="screen",
        )
    )
    activate_fr3_controller = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "control",
                    "set_controller_state",
                    "fr3_arm_controller",
                    "active",
                    "--controller-manager",
                    PathJoinSubstitution([namespace, "controller_manager"]),
                ],
                output="screen",
            )
        ],
    )
    load_controllers.append(
        ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "controller_manager",
                "spawner",
                "joint_state_broadcaster",
                "--controller-manager-timeout",
                "60",
                "--controller-manager",
                PathJoinSubstitution([namespace, "controller_manager"]),
            ],
            output="screen",
        )
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        namespace=namespace,
        parameters=[{"source_list": ["franka/joint_states", "fr3_gripper/joint_states"], "rate": 30}],
    )

    franka_robot_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["franka_robot_state_broadcaster"],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    robot_arg = DeclareLaunchArgument(
        robot_ip_parameter_name, description="Hostname or IP address of the robot."
    )

    namespace_arg = DeclareLaunchArgument(
        namespace_parameter_name, default_value="", description="Namespace for the robot."
    )
    load_gripper_arg = DeclareLaunchArgument(
        load_gripper_parameter_name,
        default_value="true",
        description="Whether to load the gripper or not (true or false)",
    )
    ee_id_arg = DeclareLaunchArgument(
        ee_id_parameter_name,
        default_value="franka_hand",
        description="The end-effector id to use. Available options: none, franka_hand, cobot_pump",
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name, default_value="false", description="Use fake hardware"
    )
    fake_sensor_commands_arg = DeclareLaunchArgument(
        fake_sensor_commands_parameter_name,
        default_value="false",
        description="Fake sensor commands. Only valid when '{}' is true".format(
            use_fake_hardware_parameter_name
        ),
    )
    pause_topic_arg = DeclareLaunchArgument(
        "pause_topic",
        default_value="/dress/motion_gate",
        description="Topic the executor pauses on ('1' = move, '0' = pause). "
        "dress_node publishes /dress/motion_gate; /engagement/status is the old raw attention label.",
    )
    continuous_execution_arg = DeclareLaunchArgument(
        "continuous_execution",
        default_value="false",
        description="Run all received waypoints as one motion, without stopping at each. "
        "Start dmp_node with continuous_execution too (study_trial.launch.py continuous:=true).",
    )
    cartesian_speed_arg = DeclareLaunchArgument(
        "cartesian_speed",
        default_value="0.03",
        description="Average end-effector speed in m/s when continuous_execution is on. "
        "Can only slow the motion down: velocity_scaling remains the upper limit.",
    )
    planning_mode_arg = DeclareLaunchArgument(
        planning_mode_parameter_name,
        default_value="cartesian",
        description="Trajectory planning mode: cartesian or planning",
    )
    planner_id_arg = DeclareLaunchArgument(
        planner_id_parameter_name,
        default_value="",
        description="MoveIt planner ID (e.g., BoltConfigDefault).",
    )
    planning_pipeline_arg = DeclareLaunchArgument(
        planning_pipeline_parameter_name,
        default_value="",
        description="MoveIt planning pipeline ID (e.g., ompl, pilz_industrial_motion_planner, descartes).",
    )
    lock_orientation_arg = DeclareLaunchArgument(
        lock_orientation_parameter_name,
        default_value="false",
        description="Whether to lock the end effector orientation to the current state",
    )

    gripper_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("franka_gripper"), "launch", "gripper.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            use_fake_hardware_parameter_name: use_fake_hardware,
            "namespace": namespace,
        }.items(),
    )

    executor_node = Node(
        package="franka_moveit_compliant_control",
        executable="cartesian_trajectory_executor",
        name="cartesian_trajectory_executor",
        namespace=namespace,
        output="screen",
        parameters=[
            {"move_group": "fr3_arm"},
            {"planning_mode": planning_mode},
            {"attention_status_topic": LaunchConfiguration("pause_topic")},
            {"continuous_execution": ParameterValue(
                LaunchConfiguration("continuous_execution"), value_type=bool)},
            {"cartesian_speed": ParameterValue(
                LaunchConfiguration("cartesian_speed"), value_type=float)},
            {"planner_id": planner_id},
            {"planning_pipeline": planning_pipeline},
            {"lock_orientation": lock_orientation},
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
    )

    return LaunchDescription(
        [
            robot_arg,
            continuous_execution_arg,
            cartesian_speed_arg,
            namespace_arg,
            load_gripper_arg,
            ee_id_arg,
            use_fake_hardware_arg,
            fake_sensor_commands_arg,
            planning_mode_arg,
            pause_topic_arg,
            planner_id_arg,
            planning_pipeline_arg,
            lock_orientation_arg,
            db_arg,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            joint_state_publisher,
            franka_robot_state_broadcaster,
            gripper_launch_file,
            executor_node,
            activate_fr3_controller,
        ]
        + load_controllers
    )
