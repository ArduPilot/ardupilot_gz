import yaml
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction,
    ExecuteProcess, RegisterEventHandler, TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
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


def _load_iris_sdf():
    pkg_gz = get_package_share_directory("ardupilot_gz_description")
    with open(os.path.join(pkg_gz, "models", "iris_with_arm", "model.sdf"), "r") as f:
        sdf = f.read()
    sdf = sdf.replace("model://iris_with_standoffs",
                       "package://ardupilot_gazebo/models/iris_with_standoffs")
    sdf = sdf.replace("model://iris_with_arm",
                       "package://ardupilot_gz_description/models/iris_with_arm")
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_path = os.environ["GZ_SIM_RESOURCE_PATH"]
        os.environ["SDF_PATH"] = os.environ.get("SDF_PATH", "") + ":" + gz_path
    return sdf


def launch_setup(context, *args, **kwargs):
    launch_rviz = LaunchConfiguration("launch_rviz")

    robot_description_full = {"robot_description": _load_iris_sdf()}

    srdf_path = os.path.join(get_package_share_directory(PKG), "srdf", "iris_with_arm.srdf")
    with open(srdf_path, "r") as f:
        srdf_content = f.read()
    robot_description_semantic = {"robot_description_semantic": srdf_content}

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

    arm_urdf_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(PKG), "urdf", "learm.urdf.xacro"]),
    ])
    arm_robot_description = {"robot_description": ParameterValue(arm_urdf_content, value_type=str)}

    ros2_controllers_path = os.path.join(
        get_package_share_directory(PKG), "config", "ros2_controllers.yaml"
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[arm_robot_description, ros2_controllers_path, {"use_sim_time": True}],
        output="screen",
        remappings=[("joint_states", "arm_mock_joint_states")],
    )

    jsb_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner",
             "joint_state_broadcaster", "--controller-manager", "/controller_manager",
             "--controller-manager-timeout", "30"],
        output="screen",
    )
    arm_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner",
             "arm_controller", "--controller-manager", "/controller_manager",
             "--controller-manager-timeout", "30"],
        output="screen",
    )
    delayed_jsb = RegisterEventHandler(event_handler=OnProcessStart(
        target_action=ros2_control_node,
        on_start=[TimerAction(period=3.0, actions=[jsb_spawner])],
    ))
    delayed_arm = RegisterEventHandler(event_handler=OnProcessStart(
        target_action=ros2_control_node,
        on_start=[TimerAction(period=6.0, actions=[arm_spawner])],
    ))

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description_full, robot_description_semantic, robot_description_kinematics,
            ompl_planning_pipeline_config, trajectory_execution,
            moveit_controllers, planning_scene_monitor,
            {"use_sim_time": True},
        ],
        remappings=[("joint_states", "arm_mock_joint_states")],
    )

    rviz = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", PathJoinSubstitution([FindPackageShare(PKG), "rviz", "iris_arm_moveit.rviz"])],
        parameters=[
            robot_description_full, robot_description_semantic,
            ompl_planning_pipeline_config, robot_description_kinematics,
            {"use_sim_time": True},
        ],
        remappings=[("joint_states", "arm_mock_joint_states")],
    )

    moveit_gz_bridge = Node(
        package=PKG,
        executable="moveit_to_gz_bridge.py",
        name="moveit_to_gz_bridge",
        output="screen",
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="arm_joint_gz_bridge",
        output="screen",
        parameters=[{"config_file": os.path.join(
            get_package_share_directory(PKG), "config", "arm_joint_bridge.yaml"
        )}],
    )

    return [
        ros2_control_node, delayed_jsb, delayed_arm,
        move_group, rviz, moveit_gz_bridge, ros_gz_bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
