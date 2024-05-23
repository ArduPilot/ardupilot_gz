# Copyright 2023 ArduPilot.org.
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
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler

from launch.conditions import IfCondition

from launch.event_handlers import OnProcessStart

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a launch description for a iris quadcopter."""
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")
    pkg_ardupilot_gz_description = get_package_share_directory(
        "ardupilot_gz_description"
    )
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")

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

    # Robot description.

    # Ensure `SDF_PATH` is populated as `sdformat_urdf`` uses this rather
    # than `GZ_SIM_RESOURCE_PATH` to locate resources.
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    # Load SDF file.
    sdf_file = os.path.join(
        pkg_ardupilot_gz_description, "models", "iris_with_lidar", "model.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()
        # print(robot_desc)

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

    # Bridge.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_project_bringup, "config", "iris_lidar_bridge.yaml"
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    # Transform - use if the model includes "gz::sim::systems::PosePublisher"
    #             and a filter is required.
    # topic_tools_tf = Node(
    #     package="topic_tools",
    #     executable="transform",
    #     arguments=[
    #         "/gz/tf",
    #         "/tf",
    #         "tf2_msgs/msg/TFMessage",
    #         "tf2_msgs.msg.TFMessage(transforms=[x for x in m.transforms if x.header.frame_id == 'odom'])",
    #         "--import",
    #         "tf2_msgs",
    #         "geometry_msgs",
    #     ],
    #     output="screen",
    #     respawn=True,
    # )

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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gz_tf", default_value="true", description="Use Gazebo TF."
            ),
            sitl_dds,
            robot_state_publisher,
            bridge,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=bridge,
                    on_start=[
                        topic_tools_tf
                    ]
                )
            ),
        ]
    )
