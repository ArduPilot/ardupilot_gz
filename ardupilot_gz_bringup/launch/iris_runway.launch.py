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

# Adapted from https://github.com/gazebosim/ros_gz_project_template
#
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch an iris quadcopter in Gazebo and Rviz."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description for a iris quadcopter."""
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_project_gazebo = get_package_share_directory("ardupilot_gz_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Iris.
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_project_bringup,
                        "launch",
                        "robots",
                        "iris.launch.py",
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
            "gz_args": "-v4 -s -r "
            f'{Path(pkg_project_gazebo) / "worlds" / "runway.sdf"}'
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_gz_sim_server")),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
        condition=IfCondition(LaunchConfiguration("use_gz_sim_gui")),
    )

    # RViz. Software GL fallback avoids GLXBadDrawable black-screen failures
    # seen on some desktop/remote-display setups.
    rviz_env = {}
    if os.getenv("OBDM_RVIZ_SOFTWARE_GL", "1") == "1":
        rviz_env["LIBGL_ALWAYS_SOFTWARE"] = "1"
        rviz_env["MESA_GL_VERSION_OVERRIDE"] = "3.3"
    qt_gl_backend = os.getenv("OBDM_RVIZ_QT_XCB_GL_INTEGRATION", "none")
    if qt_gl_backend:
        rviz_env["QT_XCB_GL_INTEGRATION"] = qt_gl_backend

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        additional_env=rviz_env,
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
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
                "rviz", default_value="true", description="Open RViz."
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=f'{Path(pkg_project_bringup) / "rviz" / "ruiz.rviz"}',
                description="Path to RViz config file.",
            ),
            gz_sim_server,
            gz_sim_gui,
            robot,
            rviz,
        ]
    )
