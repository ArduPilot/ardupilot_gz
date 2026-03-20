# Copyright 2025 ArduPilot.org.
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

"""Launch multiple vehicles in Gazebo and Rviz."""
from enum import Enum
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction

from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


class Vehicle(Enum):
    IRIS = "iris_with_gimbal"
    IRIS_LIDAR = "iris_lidar"
    WILD_THUMPER = "wildthumper"
    ALTI_TRANSITION = "alti_transition"


VEHICLE_PATHS = {
    Vehicle.IRIS: {
        "launch": "iris.launch.py",
        "rviz": "iris.rviz",
    },
    Vehicle.IRIS_LIDAR: {
        "launch": "iris_lidar.launch.py",
        "rviz": "iris_with_lidar.rviz",
    },
    Vehicle.WILD_THUMPER: {
        "launch": "wildthumper.launch.py",
        "rviz": "wildthumper.rviz",
    },
    Vehicle.ALTI_TRANSITION: {
        "launch": "alti_transition.launch.py",
        "rviz": "alti_transition.rviz",
    },
}


def get_default_launch_arguments(launch_description_source, context):
    """Retrieve default launch arguments from a LaunchDescriptionSource."""
    ild = IncludeLaunchDescription(
        launch_description_source,
    )
    ld = LaunchDescription([ild])
    default_args = {}
    for arg in ld.get_launch_arguments():
        name = arg.name
        default_value = arg.default_value[0].perform(context)
        default_args[name] = default_value
    return default_args


def generate_launch_description():
    """Generate a launch description for a iris quadcopter."""
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_project_gazebo = get_package_share_directory("ardupilot_gz_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    robots = [
        {
            "name": "drone1",
            "model": Vehicle.IRIS,
            "position": ["0.0", "0.0", "0.195", "0", "0", "1.5708"],
        },  # [x, y, z, roll, pitch, yaw]
        {
            "name": "drone2",
            "model": Vehicle.IRIS_LIDAR,
            "position": ["1.0", "0.0", "0.195", "0", "0", "1.5708"],
        },
        {
            "name": "rover1",
            "model": Vehicle.WILD_THUMPER,
            "position": ["-1.0", "0.0", "0.195", "0", "0", "1.5708"],
        },
        {
            "name": "plane1",
            "model": Vehicle.ALTI_TRANSITION,
            "position": ["-4.0", "0.0", "0.195", "0", "0", "1.5708"],
        },
    ]

    def generate_launch_actions(context: LaunchContext, *args, **kwargs):
        launch_actions = [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Run Gazebo simulation headless.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
                ),
                launch_arguments={
                    "gz_args": "-v4 -s -r "
                    f'{Path(pkg_project_gazebo) / "worlds" / "runway.sdf"}'
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
                ),
                launch_arguments={"gz_args": "-v1 -g"}.items(),
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
        ]

        for i, robot in enumerate(robots):
            instance = i
            sysid = i + 1

            name = robot["name"]
            model = robot["model"]
            position = robot["position"]

            drone_lds = PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            pkg_project_bringup,
                            "launch",
                            "robots",
                            VEHICLE_PATHS[model]["launch"],
                        ]
                    ),
                ]
            )

            # Create launch arguments, overriding defaults
            drone_launch_arguments = get_default_launch_arguments(drone_lds, context)
            drone_launch_arguments.update(
                {
                    "robot_name": name,
                    "world_name": "runway",
                    "x": position[0],
                    "y": position[1],
                    "z": position[2],
                    "R": position[3],
                    "P": position[4],
                    "Y": position[5],
                    "instance": str(instance),
                    "sysid": str(sysid),
                    "use_instance_dir": "True",
                }
            )

            # Only launch a single instance of the micro_ros_agent
            if instance > 0:
                drone_launch_arguments.update({"use_dds_agent": "False"})

            drone = IncludeLaunchDescription(
                drone_lds,
                launch_arguments=drone_launch_arguments.items(),
            )
            launch_actions.append(drone)

            rviz = Node(
                package="rviz2",
                executable="rviz2",
                namespace=name,
                arguments=[
                    "-d",
                    f'{Path(pkg_project_bringup) / "rviz" / VEHICLE_PATHS[model]["rviz"]}',
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                ],
            )
            launch_actions.append(rviz)

        return launch_actions

    return LaunchDescription([OpaqueFunction(function=generate_launch_actions)])
