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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class Vehicle(Enum):
    IRIS = "iris_with_gimbal"
    IRIS_LIDAR = "iris_lidar"
    WILD_THUMPER = "wildthumper"


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
}


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
    ]

    launch_actions = [
        DeclareLaunchArgument("rviz", default_value="true", description="Open RViz."),
        DeclareLaunchArgument(
            "gui", default_value="true", description="Run Gazebo simulation headless."
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
            ),
            launch_arguments={
                "gz_args": "-v4 -s -r "
                f'{Path(pkg_project_gazebo) / "worlds" / "iris_runway.sdf"}'
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
            ),
            launch_arguments={"gz_args": "-v4 -g"}.items(),
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
    ]

    for i, robot in enumerate(robots):
        instance = i
        sysid = i + 1

        name = robot["name"]
        model = robot["model"]
        position = robot["position"]

        drone = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ardupilot_gz_bringup"),
                            "launch",
                            "robots",
                            VEHICLE_PATHS[model]["launch"],
                        ]
                    ),
                ]
            ),
            launch_arguments={
                "name": name,
                "x": position[0],
                "y": position[1],
                "z": position[2],
                "R": position[3],
                "P": position[4],
                "Y": position[5],
                "instance": str(instance),
                "sysid": str(sysid),
            }.items(),
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

    return LaunchDescription(launch_actions)
