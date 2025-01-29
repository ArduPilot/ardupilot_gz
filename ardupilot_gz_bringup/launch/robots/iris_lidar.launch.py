# Copyright 2024 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

"""
Launch an iris quadcopter in Gazebo and Rviz.

ros2 launch ardupilot_sitl sitl_dds_udp.launch.py
transport:=udp4
port:=2019
synthetic_clock:=True
wipe:=False
model:=json
speedup:=1
slave:=0
instance:=0
defaults:=$(ros2 pkg prefix ardupilot_sitl)
          /share/ardupilot_sitl/config/default_params/gazebo-iris.parm,
          $(ros2 pkg prefix ardupilot_sitl)
          /share/ardupilot_sitl/config/default_params/dds_udp.parm
sim_address:=127.0.0.1
master:=tcp:127.0.0.1:5760
sitl:=127.0.0.1:5501
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_spawn_robot(context):
    """Return a Gazebo spawn robot launch description"""
    # Get substitutions for arguments
    name = LaunchConfiguration("name")
    pos_x = LaunchConfiguration("x")
    pos_y = LaunchConfiguration("y")
    pos_z = LaunchConfiguration("z")
    rot_r = LaunchConfiguration("R")
    rot_p = LaunchConfiguration("P")
    rot_y = LaunchConfiguration("Y")

    # spawn robot
    spawn_robot =  Node(
        package="ros_gz_sim",
        executable="create",
        namespace=name,
        arguments=[
            "-world",
            "",
            "-param",
            "",
            "-name",
            name,
            "-topic",
            "/robot_description",
            "-x",
            pos_x,
            "-y",
            pos_y,
            "-z",
            pos_z,
            "-R",
            rot_r,
            "-P",
            rot_p,
            "-Y",
            rot_y,
        ],
        output="screen",
    )
    return [spawn_robot]


def launch_state_pub_with_bridge(context):
    # Robot description and ros_gz bridge config chosen based on passed lidar_dimension argument
    lidar_dim = LaunchConfiguration("lidar_dim").perform(context)
    pkg_ardupilot_gz_description = get_package_share_directory("ardupilot_gz_description")
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    
    sdf_file = os.path.join(
        pkg_ardupilot_gz_description, "models", "iris_with_lidar", "model.sdf"
    )

    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()
        # print(robot_desc)

    ros_gz_bridge_config = "iris_3Dlidar_bridge.yaml"

    # Load SDF file and choose ros_gz bridge config based on lidar dimensions
    if lidar_dim == "2":
        log = LogInfo(msg="Using iris_with_2d_lidar_model ")
        robot_desc = robot_desc.replace("<uri>model://lidar_2d</uri>", "<uri>model://lidar_2d</uri>")
        ros_gz_bridge_config = "iris_2Dlidar_bridge.yaml"
    elif lidar_dim == "3":
        log = LogInfo(msg="Using iris_with_3d_lidar_model")
        robot_desc = robot_desc.replace("<uri>model://lidar_2d</uri>", "<uri>model://lidar_3d</uri>")
        ros_gz_bridge_config = "iris_3Dlidar_bridge.yaml"
    else:
        log = LogInfo(msg="ERROR: unknown lidar dimensions! Defaulting to 3d lidar")
        robot_desc = robot_desc.replace("<uri>model://lidar_2d</uri>", "<uri>model://lidar_3d</uri>")
        ros_gz_bridge_config = "iris_3Dlidar_bridge.yaml"


    # Publish /tf and /tf_static.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc},
            {"frame_prefix": ""},
        ],
    )

    # Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_project_bringup, "config", ros_gz_bridge_config
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    # Relay - use instead of transform when Gazebo is only publishing odom -> base_link
    topic_tools_tf = Node(
        package="topic_tools",
        executable="relay",
        arguments=[
            "/gz/tf",
            "/tf",
        ],
        output="screen",
        respawn=False,
        condition=IfCondition(LaunchConfiguration("use_gz_tf")),
    )

    event = RegisterEventHandler(
                OnProcessStart(
                    target_action=bridge,
                    on_start=[
                        topic_tools_tf
                    ]
                )
            )

    return [log, robot_state_publisher, bridge, event]

def generate_launch_arguments():
    """Generate a list of launch arguments"""
    return [
        DeclareLaunchArgument(
                "use_gz_tf", 
                default_value="true", 
                description="Use Gazebo TF."
            ),
        DeclareLaunchArgument(
                "lidar_dim", 
                default_value="3", 
                description="Whether to use a 2D or 3D lidar"
            ),
        # Gazebo model launch arguments.
        DeclareLaunchArgument(
            "model",
            default_value="iris_with_lidar",
            description="Name or filepath of the model to load.",
        ),
        DeclareLaunchArgument(
            "name",
            default_value="iris",
            description="Name for the model instance.",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="0",
            description="The intial 'x' position (m).",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="0",
            description="The intial 'y' position (m).",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0",
            description="The intial 'z' position (m).",
        ),
        DeclareLaunchArgument(
            "R",
            default_value="0",
            description="The intial roll angle (radians).",
        ),
        DeclareLaunchArgument(
            "P",
            default_value="0",
            description="The intial pitch angle (radians).",
        ),
        DeclareLaunchArgument(
            "Y",
            default_value="0",
            description="The intial yaw angle (radians).",
        ),
    ]

def generate_launch_description():
    """Generate a launch description for a iris quadrotor"""

    launch_arguments = generate_launch_arguments()

    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")

    # Include component launch files.
    sitl_dds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "sitl_dds_udp.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "transport": "udp4",
            "port": "2019",
            "synthetic_clock": "True",
            "wipe": "False",
            "model": "json",
            "speedup": "1",
            "slave": "0",
            "instance": "0",
            "defaults": os.path.join(
                pkg_ardupilot_sitl,
                "config",
                "default_params",
                "gazebo-iris.parm",
            )
            + ","
            + os.path.join(
                pkg_ardupilot_sitl,
                "config",
                "default_params",
                "dds_udp.parm",
            ),
            "sim_address": "127.0.0.1",
            "master": "tcp:127.0.0.1:5760",
            "sitl": "127.0.0.1:5501",
        }.items(),
    )

    # Ensure `SDF_PATH` is populated as `sdformat_urdf`` uses this rather
    # than `GZ_SIM_RESOURCE_PATH` to locate resources.
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    opfunc_robot_state_publisher = OpaqueFunction(function=launch_state_pub_with_bridge)
    opfunc_spawn_robot = OpaqueFunction(function=launch_spawn_robot)
    ld = LaunchDescription(launch_arguments)
    ld.add_action(sitl_dds)
    ld.add_action(opfunc_robot_state_publisher)
    ld.add_action(opfunc_spawn_robot)

    return ld
