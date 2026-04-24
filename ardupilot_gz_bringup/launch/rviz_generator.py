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
Generate an RViz config from SDF files.

Reads the same ``<sensor>`` tree that ``bridge_generator`` does and emits
an ``*.rviz`` config with one RViz display per sensor output (image,
point cloud, laser scan, etc.) plus the usual Grid/Axes/TF/RobotModel/
Odometry displays.

Topic names are relative (e.g. ``scan`` rather than ``/iris/scan``) so
the generated config works correctly when rviz2 is launched under the
robot namespace.
"""

from typing import Any, Dict, List, Optional

import yaml

from bridge_generator import parse_sensors


_LIDAR_PALETTE: List[str] = [
    "239; 83; 80",    # red
    "38; 198; 218",   # cyan
    "255; 214; 0",    # yellow
    "171; 71; 188",   # purple
    "102; 187; 106",  # green
    "255; 112; 67",   # orange
    "66; 165; 245",   # blue
    "236; 64; 122",   # pink
]


# ---------------------------------------------------------------------------
# Building blocks for the RViz display tree.
# ---------------------------------------------------------------------------

def _topic_block(name: str) -> Dict[str, Any]:
    return {
        "Depth": 5,
        "Durability Policy": "Volatile",
        "Filter size": 10,
        "History Policy": "Keep Last",
        "Reliability Policy": "Reliable",
        "Value": name,
    }


def _grid_display() -> Dict[str, Any]:
    return {
        "Alpha": 0.5,
        "Cell Size": 1,
        "Class": "rviz_default_plugins/Grid",
        "Color": "160; 160; 164",
        "Enabled": True,
        "Line Style": {"Line Width": 0.03, "Value": "Lines"},
        "Name": "Grid",
        "Normal Cell Count": 0,
        "Offset": {"X": 0, "Y": 0, "Z": 0},
        "Plane": "XY",
        "Plane Cell Count": 10,
        "Reference Frame": "<Fixed Frame>",
        "Value": True,
    }


def _axes_display() -> Dict[str, Any]:
    return {
        "Class": "rviz_default_plugins/Axes",
        "Enabled": True,
        "Length": 1.0,
        "Name": "Axes",
        "Radius": 0.05,
        "Reference Frame": "<Fixed Frame>",
        "Value": True,
    }


def _tf_display() -> Dict[str, Any]:
    return {
        "Class": "rviz_default_plugins/TF",
        "Enabled": True,
        "Frame Timeout": 15,
        "Frames": {"All Enabled": True},
        "Marker Scale": 1,
        "Name": "TF",
        "Show Arrows": True,
        "Show Axes": True,
        "Show Names": False,
        "Update Interval": 0,
        "Value": True,
    }


def _robot_model_display() -> Dict[str, Any]:
    return {
        "Alpha": 1,
        "Class": "rviz_default_plugins/RobotModel",
        "Collision Enabled": False,
        "Description File": "",
        "Description Source": "Topic",
        "Description Topic": _topic_block("robot_description"),
        "Enabled": True,
        "Links": {
            "All Links Enabled": True,
            "Expand Joint Details": False,
            "Expand Link Details": False,
            "Expand Tree": False,
            "Link Tree Style": "Links in Alphabetic Order",
        },
        "Mass Properties": {"Inertia": False, "Mass": False},
        "Name": "RobotModel",
        "TF Prefix": "",
        "Update Interval": 0,
        "Value": True,
        "Visual Enabled": True,
    }


def _odometry_display() -> Dict[str, Any]:
    return {
        "Angle Tolerance": 0.1,
        "Class": "rviz_default_plugins/Odometry",
        "Covariance": {
            "Orientation": {
                "Alpha": 0.5,
                "Color": "255; 255; 127",
                "Color Style": "Unique",
                "Frame": "Local",
                "Offset": 1,
                "Scale": 1,
                "Value": True,
            },
            "Position": {
                "Alpha": 0.3,
                "Color": "204; 51; 204",
                "Scale": 1,
                "Value": True,
            },
            "Value": True,
        },
        "Enabled": True,
        "Keep": 100,
        "Name": "Odometry",
        "Position Tolerance": 0.1,
        "Shape": {
            "Alpha": 1,
            "Axes Length": 1,
            "Axes Radius": 0.1,
            "Color": "255; 25; 255",
            "Head Length": 0.3,
            "Head Radius": 0.1,
            "Shaft Length": 1,
            "Shaft Radius": 0.05,
            "Value": "Arrow",
        },
        "Topic": _topic_block("odometry"),
        "Value": True,
    }


def _image_display(name: str, topic: str) -> Dict[str, Any]:
    return {
        "Class": "rviz_default_plugins/Image",
        "Enabled": True,
        "Max Value": 1,
        "Median window": 5,
        "Min Value": 0,
        "Name": name,
        "Normalize Range": True,
        "Topic": _topic_block(topic),
        "Value": True,
    }


def _point_cloud_display(
    name: str, topic: str, color: Optional[str] = None
) -> Dict[str, Any]:
    return {
        "Alpha": 1,
        "Autocompute Intensity Bounds": True,
        "Autocompute Value Bounds": {
            "Max Value": 10,
            "Min Value": -10,
            "Value": True,
        },
        "Axis": "Z",
        "Channel Name": "intensity",
        "Class": "rviz_default_plugins/PointCloud2",
        "Color": color if color else "255; 255; 255",
        "Color Transformer": "FlatColor" if color else "Intensity",
        "Decay Time": 0,
        "Enabled": True,
        "Invert Rainbow": False,
        "Max Color": "255; 255; 255",
        "Max Intensity": 0,
        "Min Color": "0; 0; 0",
        "Min Intensity": 0,
        "Name": name,
        "Position Transformer": "XYZ",
        "Selectable": True,
        "Size (Pixels)": 3,
        "Size (m)": 0.01,
        "Style": "Flat Squares",
        "Topic": _topic_block(topic),
        "Use Fixed Frame": True,
        "Use rainbow": True,
        "Value": True,
    }


def _laser_scan_display(
    name: str, topic: str, color: Optional[str] = None
) -> Dict[str, Any]:
    return {
        "Alpha": 1,
        "Autocompute Intensity Bounds": True,
        "Autocompute Value Bounds": {
            "Max Value": 10,
            "Min Value": -10,
            "Value": True,
        },
        "Axis": "Z",
        "Channel Name": "intensity",
        "Class": "rviz_default_plugins/LaserScan",
        "Color": color if color else "255; 255; 255",
        "Color Transformer": "FlatColor" if color else "Intensity",
        "Decay Time": 0,
        "Enabled": True,
        "Invert Rainbow": False,
        "Max Color": "255; 255; 255",
        "Max Intensity": 0,
        "Min Color": "0; 0; 0",
        "Min Intensity": 0,
        "Name": name,
        "Position Transformer": "XYZ",
        "Selectable": True,
        "Size (Pixels)": 3,
        "Size (m)": 0.01,
        "Style": "Flat Squares",
        "Topic": _topic_block(topic),
        "Use Fixed Frame": True,
        "Use rainbow": True,
        "Value": True,
    }


# ---------------------------------------------------------------------------
# Sensor-type -> displays mapping.
# ---------------------------------------------------------------------------

def _sensor_displays(
    sensor: Dict[str, str], color: Optional[str] = None
) -> List[Dict[str, Any]]:
    name = sensor["sensor"]
    kind = sensor["type"]
    if kind in ("camera", "rgbd_camera", "depth_camera"):
        displays = [_image_display(f"{name}/image", f"{name}/image")]
        if kind in ("rgbd_camera", "depth_camera"):
            displays.append(
                _image_display(f"{name}/depth_image", f"{name}/depth_image")
            )
            displays.append(
                _point_cloud_display(f"{name}/points", f"{name}/points")
            )
        return displays
    if kind in ("gpu_lidar", "lidar"):
        return [
            _laser_scan_display(f"{name}/scan", f"{name}/scan", color),
            _point_cloud_display(
                f"{name}/points", f"{name}/scan/points", color
            ),
        ]
    return []


# ---------------------------------------------------------------------------
# Top-level assembly.
# ---------------------------------------------------------------------------

def _panels() -> List[Dict[str, Any]]:
    return [
        {
            "Class": "rviz_common/Displays",
            "Help Height": 0,
            "Name": "Displays",
            "Property Tree Widget": {
                "Expanded": ["/Global Options1", "/Status1"],
                "Splitter Ratio": 0.5,
            },
            "Tree Height": 587,
        },
        {"Class": "rviz_common/Selection", "Name": "Selection"},
        {
            "Class": "rviz_common/Tool Properties",
            "Expanded": ["/2D Goal Pose1", "/Publish Point1"],
            "Name": "Tool Properties",
            "Splitter Ratio": 0.5886790156364441,
        },
        {
            "Class": "rviz_common/Views",
            "Expanded": ["/Current View1"],
            "Name": "Views",
            "Splitter Ratio": 0.5,
        },
        {
            "Class": "rviz_common/Time",
            "Experimental": False,
            "Name": "Time",
            "SyncMode": 0,
            "SyncSource": "",
        },
    ]


def _tools() -> List[Dict[str, Any]]:
    return [
        {"Class": "rviz_default_plugins/Interact", "Hide Inactive Objects": True},
        {"Class": "rviz_default_plugins/MoveCamera"},
        {"Class": "rviz_default_plugins/Select"},
        {"Class": "rviz_default_plugins/FocusCamera"},
        {
            "Class": "rviz_default_plugins/Measure",
            "Line color": "128; 128; 0",
        },
        {
            "Class": "rviz_default_plugins/SetInitialPose",
            "Covariance x": 0.25,
            "Covariance y": 0.25,
            "Covariance yaw": 0.06853891909122467,
            "Topic": _topic_block("initialpose"),
        },
        {
            "Class": "rviz_default_plugins/SetGoal",
            "Topic": _topic_block("goal_pose"),
        },
        {
            "Class": "rviz_default_plugins/PublishPoint",
            "Single click": True,
            "Topic": _topic_block("clicked_point"),
        },
    ]


def _views() -> Dict[str, Any]:
    return {
        "Current": {
            "Class": "rviz_default_plugins/Orbit",
            "Distance": 9.58,
            "Enable Stereo Rendering": {
                "Stereo Eye Separation": 0.06,
                "Stereo Focal Distance": 1,
                "Swap Stereo Eyes": False,
                "Value": False,
            },
            "Focal Point": {"X": 0.0, "Y": 0.0, "Z": 0.0},
            "Focal Shape Fixed Size": True,
            "Focal Shape Size": 0.05,
            "Invert Z Axis": False,
            "Name": "Current View",
            "Near Clip Distance": 0.01,
            "Pitch": 0.69,
            "Target Frame": "<Fixed Frame>",
            "Value": "Orbit (rviz)",
            "Yaw": 5.1,
        },
        "Saved": None,
    }


# Qt-serialised dock layout. rviz2 crashes in libX11 / ogre1 on resize
# when an Image/Camera display has to materialise a dock at runtime with
# no saved state to restore. Shipping a canonical blob with an "Image" dock
# slot is enough to stabilise the startup sequence even for displays whose
# names are not literally encoded in the blob - Qt docks everything
# else tabbed alongside.
_QMAINWINDOW_STATE_BLOB = (
    "000000ff00000000fd0000000400000000000001630000029afc0200000009"
    "fb0000001200530065006c0065006300740069006f006e00000001e10000009b"
    "0000006200fffffffb0000001e0054006f006f006c002000500072006f007000"
    "650072007400690065007302000001ed000001df00000185000000a3fb000000"
    "120056006900650077007300200054006f006f02000001df0000021100000185"
    "00000122fb000000200054006f006f006c002000500072006f00700065007200"
    "74006900650073003203000002880000011d000002210000017afb0000001000"
    "44006900730070006c006100790073010000002c0000016f000000e300ffffff"
    "fb0000002000730065006c0065006300740069006f006e0020006200750066006"
    "6006500720200000138000000aa0000023a00000294fb000000140057006900640"
    "06500530074006500720065006f02000000e6000000d2000003ee0000030bfb00"
    "00000c004b0069006e0065006300740200000186000001060000030c00000261f"
    "b0000000a0049006d006100670065010000019c0000012a0000003000ffffff00"
    "0000010000010f0000029bfc0200000003fb0000001e0054006f006f006c00200"
    "0500072006f00700065007200740069006500730100000041000000780000000000"
    "000000fb0000000a00560069006500770073000000002c0000029b000000c600ff"
    "fffffb0000001200530065006c0065006300740069006f006e010000025a000000"
    "b200000000000000000000000200000490000000a9fc0100000001fb0000000a00"
    "560069006500770073030000004e00000080000002e10000019700000003000004"
    "b00000003efc0100000002fb0000000800540069006d00650100000000000004b0"
    "0000023d00fffffffb0000000800540069006d006501000000000000045000000000"
    "000000000000034c0000029a00000004000000040000000800000008fc000000010"
    "0000002000000010000000a0054006f006f006c00730100000000ffffffff000000"
    "0000000000"
)


def _window_geometry(extra_docks: List[str] = ()) -> Dict[str, Any]:
    geom: Dict[str, Any] = {
        "Displays": {"collapsed": False},
        "Height": 800,
        "Hide Left Dock": False,
        "Hide Right Dock": False,
        "QMainWindow State": _QMAINWINDOW_STATE_BLOB,
        "Selection": {"collapsed": False},
        "Time": {"collapsed": False},
        "Tool Properties": {"collapsed": False},
        "Views": {"collapsed": False},
        "Width": 1200,
        "X": 0,
        "Y": 38,
    }
    for dock in extra_docks:
        geom[dock] = {"collapsed": False}
    return geom


def generate_rviz_config(
    sdf_files: List[str],
    output_path: str,
    fixed_frame: str = "odom",
) -> str:
    """Parse sensors from ``sdf_files`` and emit an rviz2 config yaml."""
    all_sensors: List[Dict[str, str]] = []
    seen = set()
    for sdf in sdf_files:
        for s in parse_sensors(sdf):
            key = (s["link"], s["sensor"], s["type"])
            if key in seen:
                continue
            seen.add(key)
            all_sensors.append(s)

    robot_group_displays: List[Dict[str, Any]] = [
        _grid_display(),
        _axes_display(),
        _robot_model_display(),
        _tf_display(),
        _odometry_display(),
    ]

    # If there's more than one lidar, give each a distinct flat colour so
    # their point clouds/scans are visually separable in rviz. A single
    # lidar keeps the default intensity-based colouring.
    lidar_sensors = [
        s for s in all_sensors if s["type"] in ("gpu_lidar", "lidar")
    ]
    lidar_colors: Dict[str, str] = {}
    if len(lidar_sensors) > 1:
        for idx, s in enumerate(lidar_sensors):
            lidar_colors[s["sensor"]] = _LIDAR_PALETTE[idx % len(_LIDAR_PALETTE)]

    sensor_group_displays: List[Dict[str, Any]] = []
    image_dock_names: List[str] = []
    for sensor in all_sensors:
        color = lidar_colors.get(sensor["sensor"])
        for display in _sensor_displays(sensor, color):
            sensor_group_displays.append(display)
            if display["Class"] == "rviz_default_plugins/Image":
                image_dock_names.append(display["Name"])

    displays: List[Dict[str, Any]] = [
        {
            "Class": "rviz_common/Group",
            "Displays": robot_group_displays,
            "Enabled": True,
            "Name": "Robot",
        }
    ]
    if sensor_group_displays:
        displays.append(
            {
                "Class": "rviz_common/Group",
                "Displays": sensor_group_displays,
                "Enabled": True,
                "Name": "Sensors",
            }
        )

    config: Dict[str, Any] = {
        "Panels": _panels(),
        "Visualization Manager": {
            "Class": "",
            "Displays": displays,
            "Enabled": True,
            "Global Options": {
                "Background Color": "48; 48; 48",
                "Fixed Frame": fixed_frame,
                "Frame Rate": 30,
            },
            "Name": "root",
            "Tools": _tools(),
            "Transformation": {
                "Current": {"Class": "rviz_default_plugins/TF"},
            },
            "Value": True,
            "Views": _views(),
        },
        "Window Geometry": _window_geometry(image_dock_names),
    }

    header = (
        "# This file is auto-generated by rviz_generator.py.\n"
        "# Do not edit: changes are overwritten each time the launch runs.\n"
    )
    with open(output_path, "w") as f:
        f.write(header)
        yaml.safe_dump(config, f, default_flow_style=False, sort_keys=False)
    return output_path
