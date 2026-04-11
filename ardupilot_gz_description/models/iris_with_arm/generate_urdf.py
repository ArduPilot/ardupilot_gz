"""
Generate a urdf robot description from sdf for the iris with arm.

This script used the ros2 package sdf_to_urdf to generate a urdf file from
a sdf robot description.

Usage

python ./generate_urdf.py

Notes

Like `sdformat_urdf` which it uses, `sdf_to_urdf` resolves dependent packages
using the ROS 2 style `package://` prefix rather than Gazebo's `model://`.

The output file is named `model.urdf`. This file should not be edited.
"""

import os
import subprocess
import tempfile
import xacro

from copy import deepcopy
from pathlib import Path
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory


# robot name
robot_name = "iris_with_arm"

# xacro parameters
prefix = "arm_"
use_sim = "true"
use_topic_hardware_interface = "true"


def generate_urdf_model():
    ardupilot_gz_description_path = os.path.join(
        get_package_share_directory("ardupilot_gz_description")
    )

    # input model sdf file
    model_sdf_file = os.path.join(
        ardupilot_gz_description_path, "models", robot_name, "model.sdf"
    )

    # input xacro file - combines ros2_control element with xacro generated
    # from the input model sdf file
    model_xacro_file = os.path.join(
        ardupilot_gz_description_path, "models", robot_name, "model.urdf.xacro"
    )

    # initial position file for the arm
    initial_positions_file = os.path.join(
        ardupilot_gz_description_path,
        "models",
        "so_arm_100",
        "initial_positions.yaml",
    )

    # xacro generated from input model sdf file
    iris_with_arm_xacro_file = os.path.join(
        ardupilot_gz_description_path,
        "models",
        robot_name,
        "iris_with_arm.urdf.xacro",
    )

    # output model urdf file
    model_urdf_file = os.path.join(
        ardupilot_gz_description_path, "models", robot_name, "model.urdf"
    )

    # open orignal sdf model
    with open(model_sdf_file) as file:
        model_sdf = file.read()

    # replace model:// prefix with package location
    replace_str = f"model://iris_with_standoffs"
    with_str = f"package://ardupilot_gazebo/models/iris_with_standoffs"
    model_sdf = model_sdf.replace(replace_str, with_str)

    replace_str = f"model://so_arm_100"
    with_str = f"package://ardupilot_gz_description/models/so_arm_100"
    model_sdf = model_sdf.replace(replace_str, with_str)

    # write sdf to temporary file needed by sdf_to_urdf
    temp_sdf_file = tempfile.NamedTemporaryFile(delete=False, suffix=".sdf")
    temp_sdf_file_name = temp_sdf_file.name

    with open(temp_sdf_file_name, "w") as file:
        file.write(model_sdf)

    # create temporary urdf file needed by xacro
    temp_urdf_file = tempfile.NamedTemporaryFile(delete=False, suffix=".urdf")
    temp_urdf_file_name = temp_urdf_file.name

    # convert from sdf to urdf, output is to `temp_urdf_file_name`
    with subprocess.Popen(
        [
            "ros2",
            "run",
            "sdf_to_urdf",
            "sdf_to_urdf",
            f"{temp_sdf_file_name}",
            f"{temp_urdf_file_name}",
        ],
        stdout=subprocess.PIPE,
    ) as proc:
        print(proc.stdout.read().decode())

    # convert urdf to xacro
    with open(temp_urdf_file_name, "r") as file:
        urdf_doc = ET.fromstring(file.read())

    # create xacro document
    ET.register_namespace("xacro", "http://www.ros.org/wiki/xacro")
    xacro_doc = ET.Element("{http://www.ros.org/wiki/xacro}robot")
    xacro_macro = ET.SubElement(xacro_doc, "{http://www.ros.org/wiki/xacro}macro")
    xacro_macro.attrib["name"] = robot_name
    xacro_macro.attrib["params"] = ""

    # copy elements from the original robot model
    for child in urdf_doc:
        xacro_macro.append(deepcopy(child))

    # save intermediate xacro to local file
    with open(iris_with_arm_xacro_file, "w") as file:
        ET.indent(xacro_doc, "    ")
        file.write(ET.tostring(xacro_doc).decode())

    model_urdf_doc = ET.fromstring(
        xacro.process_file(
            model_xacro_file,
            mappings={
                "prefix": prefix,
                "initial_positions_file": str(initial_positions_file),
                "use_sim": use_sim,
                "use_topic_hardware_interface": use_topic_hardware_interface,
            },
        ).toxml()
    )

    # save model xacro to file
    with open(model_urdf_file, "w") as file:
        ET.indent(model_urdf_doc, "    ")
        file.write(ET.tostring(model_urdf_doc).decode() + "\n")


def main(args=None):
    generate_urdf_model()


if __name__ == "__main__":
    main()
