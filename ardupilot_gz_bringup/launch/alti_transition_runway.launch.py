# Copyright 2026 ArduPilot.org.
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

"""Launch an alti transition quadplane in Gazebo and Rviz."""
import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


def cleanup_stale_processes(context, *args, **kwargs):
    """Stop stale simulation processes that hold the fixed launch ports."""
    cleanup_cmd = (
        "pkill -9 -f '/home/j/.local/bin/mavproxy.py' || true; "
        "pkill -9 -f 'arduplane --model json' || true; "
        "pkill -9 -f 'micro_ros_agent' || true; "
        "pkill -9 -f 'parameter_bridge' || true; "
        "pkill -9 -f 'gz sim' || true"
    )
    subprocess.run(["bash", "-lc", cleanup_cmd], check=False)
    return []


def generate_launch_description():
    """Generate a launch description for a alti transition quadplane."""
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_project_gazebo = get_package_share_directory("ardupilot_gz_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Robot.
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_project_bringup,
                        "launch",
                        "robots",
                        "alti_transition.launch.py",
                    ]
                ),
            ]
        ),
        condition=IfCondition(LaunchConfiguration("spawn_robot")),
    )

    # Gazebo.
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": "-v1 -s -r "
            f'{Path(pkg_project_gazebo) / "worlds" / "runway.sdf"}'
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_gz_sim_server")),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={"gz_args": "-v1 -g"}.items(),
        condition=IfCondition(LaunchConfiguration("use_gz_sim_gui")),
    )

    # RViz.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace="alti_transition",
        arguments=["-d", f'{Path(pkg_project_bringup) / "rviz" / "alti_transition.rviz"}'],
        condition=IfCondition(LaunchConfiguration("rviz")),
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    return LaunchDescription(
        [
            OpaqueFunction(function=cleanup_stale_processes),
            SetEnvironmentVariable(name="ROS_DOMAIN_ID", value="0"),
            SetEnvironmentVariable(name="GZ_VERSION", value="harmonic"),
            SetEnvironmentVariable(
                name="DISPLAY", value=os.environ.get("DISPLAY", ":0")
            ),
            SetEnvironmentVariable(
                name="XAUTHORITY", value=os.environ.get("XAUTHORITY", "")
            ),
            SetEnvironmentVariable(name="QT_X11_NO_MITSHM", value="1"),
            DeclareLaunchArgument(
                "use_gz_sim_server",
                default_value="true",
                description="Run the Gazebo server.",
            ),
            DeclareLaunchArgument(
                "use_gz_sim_gui",
                default_value="true",
                description="Run the Gazebo GUI.",
            ),
            DeclareLaunchArgument(
                "spawn_robot",
                default_value="true",
                description="Spawn the robot and start SITL+ROS.",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="false", description="Open RViz."
            ),
            gz_sim_server,
            gz_sim_gui,
            robot,
            rviz,
        ]
    )
