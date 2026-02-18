import yaml
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, ExecuteProcess,
    RegisterEventHandler, TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

PKG = "ardupilot_gz_arm_moveit"


def load_yaml(package_name, file_path):
    full_path = os.path.join(get_package_share_directory(package_name), file_path)
    try:
        with open(full_path) as f:
            return yaml.safe_load(f)
    except OSError:
        return None


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(PKG), "urdf", "learm.urdf.xacro"]),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(PKG), "srdf", "learm.srdf.xacro"]),
    ])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    robot_description_kinematics = {"robot_description_kinematics": load_yaml(PKG, "config/kinematics.yaml")}

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters":
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(load_yaml(PKG, "config/ompl_planning.yaml"))

    moveit_controllers = {
        "moveit_simple_controller_manager": load_yaml(PKG, "config/controllers.yaml"),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    ros2_controllers_path = os.path.join(
        get_package_share_directory(PKG), "config", "ros2_controllers.yaml"
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )

    jsb_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner",
             "joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    arm_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner",
             "arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    delayed_jsb = RegisterEventHandler(event_handler=OnProcessStart(
        target_action=ros2_control_node,
        on_start=[TimerAction(period=2.0, actions=[jsb_spawner])],
    ))
    delayed_arm = RegisterEventHandler(event_handler=OnProcessStart(
        target_action=ros2_control_node,
        on_start=[TimerAction(period=3.0, actions=[arm_spawner])],
    ))

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="virtual_joint_broadcaster_0",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "arm_base_link"],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description, robot_description_semantic, robot_description_kinematics,
            ompl_planning_pipeline_config, trajectory_execution,
            moveit_controllers, planning_scene_monitor,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", PathJoinSubstitution([FindPackageShare(PKG), "rviz", "view_robot.rviz"])],
        parameters=[
            robot_description, robot_description_semantic,
            ompl_planning_pipeline_config, robot_description_kinematics,
        ],
    )

    return [
        static_tf, robot_state_pub, ros2_control_node,
        delayed_jsb, delayed_arm, move_group, rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
