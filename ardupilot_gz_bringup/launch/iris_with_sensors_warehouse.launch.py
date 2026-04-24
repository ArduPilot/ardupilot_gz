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

"""Launch the iris_with_sensors quadcopter in the warehouse world."""
import importlib.util
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import List

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


_SENSOR_XACRO_MODELS = ("lidar", "camera", "depth_camera", "rgbd_camera")


def _load_module(module_name: str):
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    module_path = os.path.join(
        pkg_project_bringup, "launch", f"{module_name}.py"
    )
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _xacro_to_sdf(xacro_path: str) -> str:
    sdf_path = xacro_path.replace(".xacro", ".sdf")
    subprocess.run(
        ["ros2", "run", "xacro", "xacro", "-o", sdf_path, xacro_path],
        check=True,
    )
    return sdf_path


def _build_rviz_node(context: LaunchContext):
    """Regenerate the rviz config from the sensor SDFs and launch rviz2."""
    del context  # unused; OpaqueFunction passes it by convention.
    pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")
    pkg_project_description = get_package_share_directory("ardupilot_gz_description")

    sensor_sdfs: List[str] = []
    for model_name in _SENSOR_XACRO_MODELS:
        xacro_file = os.path.join(
            pkg_project_description, "models", model_name, "model.xacro"
        )
        sensor_sdfs.append(_xacro_to_sdf(xacro_file))

    base_sdf = os.path.join(
        pkg_ardupilot_gazebo, "models", "iris_with_standoffs", "model.sdf"
    )

    # bridge_generator must be importable before rviz_generator imports it.
    _load_module("bridge_generator")
    rviz_generator = _load_module("rviz_generator")

    rviz_tmp = tempfile.NamedTemporaryFile(
        delete=False, suffix="_iris_with_sensors.rviz"
    )
    rviz_tmp.close()
    rviz_generator.generate_rviz_config([base_sdf] + sensor_sdfs, rviz_tmp.name)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration("namespace"),
        arguments=["-d", rviz_tmp.name],
        condition=IfCondition(LaunchConfiguration("rviz")),
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )
    return [rviz]


def generate_launch_description():
    """Generate a launch description for the iris_with_sensors quadcopter."""
    pkg_project_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_project_gazebo = get_package_share_directory("ardupilot_gz_gazebo")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Iris with xacro-generated sensors.
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_project_bringup,
                        "launch",
                        "robots",
                        "iris_with_sensors.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "world_name": LaunchConfiguration("world_name"),
            "robot_name": LaunchConfiguration("robot_name"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("spawn_robot")),
    )

    # Gazebo.
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r "
            + f'{Path(pkg_project_gazebo) / "worlds" / "warehouse.sdf"}'
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

    rviz = OpaqueFunction(function=_build_rviz_node)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Robot namespace.",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="warehouse",
                description="Name for the world instance.",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="iris",
                description="Name for the model instance.",
            ),
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
            gz_sim_server,
            gz_sim_gui,
            robot,
            rviz,
        ]
    )
