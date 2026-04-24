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

"""
Generate a ros_gz_bridge config (bridge.yaml) from SDF files.

Scans one or more SDF files for ``<sensor>`` elements, determines the
owning link for each sensor, and emits the corresponding bridge entries
for the ros_gz parameter bridge.

This lets the same pipeline that xacro-generates sensor SDFs also
(re)generate a bridge config so the two stay in sync automatically.
"""

import re
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple

# Base entries that are not tied to a specific sensor (clock, odometry,
# joint_states, TF, battery). The strings ``{{ world_name }}`` and
# ``{{ robot_name }}`` are substituted at launch time by
# ``robot.launch.py`` (``replace_robot_name``).
_BASE_ENTRIES: List[Dict[str, str]] = [
    {
        "ros_topic_name": "clock",
        "gz_topic_name": "/clock",
        "ros_type_name": "rosgraph_msgs/msg/Clock",
        "gz_type_name": "gz.msgs.Clock",
    },
    {
        "ros_topic_name": "joint_states",
        "gz_topic_name": "/world/{{ world_name }}/model/{{ robot_name }}/joint_state",
        "ros_type_name": "sensor_msgs/msg/JointState",
        "gz_type_name": "gz.msgs.Model",
    },
    {
        "ros_topic_name": "odometry",
        "gz_topic_name": "/model/{{ robot_name }}/odometry",
        "ros_type_name": "nav_msgs/msg/Odometry",
        "gz_type_name": "gz.msgs.Odometry",
    },
    {
        "ros_topic_name": "gz/tf",
        "gz_topic_name": "/model/{{ robot_name }}/pose",
        "ros_type_name": "tf2_msgs/msg/TFMessage",
        "gz_type_name": "gz.msgs.Pose_V",
    },
    {
        "ros_topic_name": "gz/tf_static",
        "gz_topic_name": "/model/{{ robot_name }}/pose_static",
        "ros_type_name": "tf2_msgs/msg/TFMessage",
        "gz_type_name": "gz.msgs.Pose_V",
    },
    {
        "ros_topic_name": "battery",
        "gz_topic_name": "/model/{{ robot_name }}/battery/lipo_3500mAh/state",
        "ros_type_name": "sensor_msgs/msg/BatteryState",
        "gz_type_name": "gz.msgs.BatteryState",
    },
]

# Maps an SDF sensor ``type`` to the list of outputs we want to bridge.
# Each output is (gz_suffix, ros_topic_suffix, ros_type, gz_type).
#
# gz_suffix: trailing component in the gz topic path,
#   `/world/<w>/model/<r>/link/<link>/sensor/<sensor>/<gz_suffix>`
# ros_topic_suffix: ros topic leaf name. For single-output sensors
#   (imu, magnetometer, ...) this is the bare ros topic name; for
#   multi-output sensors it is combined with the sensor name as
#   ``<sensor_name>/<suffix>``.
_SENSOR_OUTPUTS: Dict[str, List[Tuple[str, str, str, str]]] = {
    "imu": [
        ("imu", "imu", "sensor_msgs/msg/Imu", "gz.msgs.IMU"),
    ],
    "magnetometer": [
        (
            "magnetometer",
            "magnetometer",
            "sensor_msgs/msg/MagneticField",
            "gz.msgs.Magnetometer",
        ),
    ],
    "air_pressure": [
        (
            "air_pressure",
            "air_pressure",
            "sensor_msgs/msg/FluidPressure",
            "gz.msgs.FluidPressure",
        ),
    ],
    "altimeter": [
        (
            "altimeter",
            "altimeter",
            "geometry_msgs/msg/Vector3Stamped",
            "gz.msgs.Altimeter",
        ),
    ],
    "air_speed": [
        (
            "air_speed",
            "air_speed",
            "geometry_msgs/msg/Vector3Stamped",
            "gz.msgs.AirSpeedSensor",
        ),
    ],
    "navsat": [
        (
            "navsat",
            "navsat",
            "sensor_msgs/msg/NavSatFix",
            "gz.msgs.NavSat",
        ),
        (
            "navsat",
            "gpsfix",
            "gps_msgs/msg/GPSFix",
            "gz.msgs.NavSat",
        ),
    ],
    "camera": [
        ("image", "image", "sensor_msgs/msg/Image", "gz.msgs.Image"),
        (
            "camera_info",
            "camera_info",
            "sensor_msgs/msg/CameraInfo",
            "gz.msgs.CameraInfo",
        ),
    ],
    "depth_camera": [
        ("image", "image", "sensor_msgs/msg/Image", "gz.msgs.Image"),
        (
            "depth_image",
            "depth_image",
            "sensor_msgs/msg/Image",
            "gz.msgs.Image",
        ),
        (
            "camera_info",
            "camera_info",
            "sensor_msgs/msg/CameraInfo",
            "gz.msgs.CameraInfo",
        ),
        (
            "points",
            "points",
            "sensor_msgs/msg/PointCloud2",
            "gz.msgs.PointCloudPacked",
        ),
    ],
    "rgbd_camera": [
        ("image", "image", "sensor_msgs/msg/Image", "gz.msgs.Image"),
        (
            "depth_image",
            "depth_image",
            "sensor_msgs/msg/Image",
            "gz.msgs.Image",
        ),
        (
            "camera_info",
            "camera_info",
            "sensor_msgs/msg/CameraInfo",
            "gz.msgs.CameraInfo",
        ),
        (
            "points",
            "points",
            "sensor_msgs/msg/PointCloud2",
            "gz.msgs.PointCloudPacked",
        ),
    ],
    "gpu_lidar": [
        ("scan", "scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
        (
            "scan/points",
            "scan/points",
            "sensor_msgs/msg/PointCloud2",
            "gz.msgs.PointCloudPacked",
        ),
    ],
    "lidar": [
        ("scan", "scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
        (
            "scan/points",
            "scan/points",
            "sensor_msgs/msg/PointCloud2",
            "gz.msgs.PointCloudPacked",
        ),
    ],
}

# Sensor types whose single output becomes an un-namespaced ros topic
# (e.g. ``imu`` rather than ``imu_sensor/imu``).
_BARE_TOPIC_TYPES = {
    "imu",
    "magnetometer",
    "air_pressure",
    "altimeter",
    "air_speed",
    "navsat",
}


def _strip_ns(tag: str) -> str:
    """Strip XML namespace from a tag name."""
    return tag.split("}", 1)[1] if "}" in tag else tag


def parse_sensors(sdf_path: str) -> List[Dict[str, str]]:
    """Return a list of ``{link, sensor, type}`` dicts for an SDF file."""
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    sensors: List[Dict[str, str]] = []

    def walk(node: ET.Element, current_link: str = "") -> None:
        tag = _strip_ns(node.tag)
        if tag == "link":
            current_link = node.get("name", current_link)
        if tag == "sensor":
            sensor_name = node.get("name", "")
            sensor_type = node.get("type", "")
            if current_link and sensor_name and sensor_type:
                sensors.append(
                    {
                        "link": current_link,
                        "sensor": sensor_name,
                        "type": sensor_type,
                    }
                )
            # Sensors don't contain nested sensors we care about.
            return
        for child in node:
            walk(child, current_link)

    walk(root)
    return sensors


def _short_name(sensor_name: str) -> str:
    """Strip a trailing ``_sensor`` suffix, e.g. ``imu_sensor`` -> ``imu``."""
    return re.sub(r"_sensor$", "", sensor_name)


def build_entries(sensors: List[Dict[str, str]]) -> List[Dict[str, str]]:
    """Convert parsed sensors into bridge yaml entries."""
    entries: List[Dict[str, str]] = list(_BASE_ENTRIES)
    seen_ros_topics = {e["ros_topic_name"] for e in entries}

    for sensor in sensors:
        outputs = _SENSOR_OUTPUTS.get(sensor["type"])
        if not outputs:
            continue

        link = sensor["link"]
        sensor_name = sensor["sensor"]
        gz_base = (
            "/world/{{ world_name }}/model/{{ robot_name }}"
            f"/link/{link}/sensor/{sensor_name}"
        )

        bare = sensor["type"] in _BARE_TOPIC_TYPES
        short = _short_name(sensor_name)

        for gz_suffix, ros_suffix, ros_type, gz_type in outputs:
            if bare:
                # Prefer the stripped sensor name (imu_sensor -> imu) when
                # it is distinct, otherwise fall back to the ros suffix.
                ros_topic = short if short and short != sensor_name else ros_suffix
            else:
                ros_topic = f"{sensor_name}/{ros_suffix}"

            # Skip exact duplicates so multiple imu sensors don't collide.
            if ros_topic in seen_ros_topics:
                continue
            seen_ros_topics.add(ros_topic)

            entries.append(
                {
                    "ros_topic_name": ros_topic,
                    "gz_topic_name": f"{gz_base}/{gz_suffix}",
                    "ros_type_name": ros_type,
                    "gz_type_name": gz_type,
                }
            )

    return entries


def _dump_yaml(entries: List[Dict[str, str]]) -> str:
    """Render entries as a ros_gz_bridge compatible yaml document."""
    lines = [
        "# This file is auto-generated by bridge_generator.py.",
        "# Do not edit: changes are overwritten each time the launch runs.",
        "---",
    ]
    for i, entry in enumerate(entries):
        prefix = "- "
        for key in (
            "ros_topic_name",
            "gz_topic_name",
            "ros_type_name",
            "gz_type_name",
        ):
            value = entry[key]
            lines.append(f'{prefix}{key}: "{value}"')
            prefix = "  "
        lines.append("  direction: GZ_TO_ROS")
        if i != len(entries) - 1:
            lines.append("")
    return "\n".join(lines) + "\n"


def generate_bridge_yaml(sdf_files: List[str], output_path: str) -> str:
    """Parse ``sdf_files``, write a bridge yaml to ``output_path`` and return it."""
    all_sensors: List[Dict[str, str]] = []
    seen = set()
    for sdf in sdf_files:
        for s in parse_sensors(sdf):
            key = (s["link"], s["sensor"], s["type"])
            if key in seen:
                continue
            seen.add(key)
            all_sensors.append(s)
    entries = build_entries(all_sensors)
    content = _dump_yaml(entries)
    with open(output_path, "w") as f:
        f.write(content)
    return output_path
