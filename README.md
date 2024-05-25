# ardupilot_gz

This project contains ROS 2 packages for simulating models controlled
by ArduPilot SITL with DDS support in Gazebo.

The project is adapted from the [`ros_gz_project_template`](https://github.com/gazebosim/ros_gz_project_template) project.

## Included packages

* `ardupilot_gz_description` - Contains the SDFormat description of the simulated
  system.

* `ardupilot_gz_gazebo` - Contains Gazebo specific code such as system plugins.

* `ardupilot_gz_application` - Contains ROS 2 specific code and configuration.

* `ardupilot_gz_bringup` - Contains launch files and high level utilities.


## Prerequisites

- Install [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
- Install [Gazebo Harmonic (recommended)](https://gazebosim.org/docs/harmonic) or [Gazebo Garden](https://gazebosim.org/docs/garden)
- Follow the [`Installing Build Dependencies`](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_DDS#installing-build-dependencies) section of `AP_DDS`'s README

## Install

#### 1. Create a workspace folder

```bash
mkdir -p ~/ros2_ws/src
```

#### 2. Get the project source

```bash
cd ~/ros2_ws
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src
```

#### 3. Set the Gazebo version to Harmonic or Garden:

It is recommended to put this in your `~/.bashrc` or equivalent file.

```bash
export GZ_VERSION=harmonic
```

#### 4. Update ROS dependencies

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y
```

#### 5. Build

```bash
cd ~/ros2_ws
colcon build
```

#### 6. Test

```bash
source ./install/setup.bash
colcon test --packages-select ardupilot_sitl ardupilot_dds_tests ardupilot_gazebo ardupilot_gz_applications ardupilot_gz_description ardupilot_gz_gazebo ardupilot_gz_bringup
colcon test-result --all --verbose
```

## Usage

#### 1. Source the workspace

```bash
source ~/ros2_ws/install/setup.sh
```

#### 2. Launch the simulation

```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py rviz:=true use_gz_tf:=true
```

#### 3. Launch a GCS (MAVPorxy)

```bash
mavproxy.py --master udp:127.0.0.1:14550  --console --map
```

#### 4. Inspect topics

```bash
$ ros2 topic list
/ap/battery/battery0
/ap/clock
/ap/navsat/navsat0
/ap/tf_static
/clicked_point
/clock
/goal_pose
/initialpose
/iris/odometry
/joint_states
/parameter_events
/robot_description
/rosout
/tf
/tf_static
```

## Advanced variations

Here's a list of all the additional use cases from this repository:

### 1. Iris Maze

This simulation has an iris copter equipped with a 360 degrees 2d lidar in a maze world.

```bash
ros2 launch ardupilot_gz_bringup iris_maze.launch.py rviz:=true use_gz_tf:=true
```

### 2. Wild Thumper playpen

This simulation includes a skid-steer rover equipped with a 2d lidar in a playpen world.


```bash
ros2 launch ardupilot_gz_bringup wildthumper_playpen.launch.py rviz:=true use_gz_tf:=true
```


## Notes

### 1. Additional dependencies

`ros_gz` has a dependency on `gps_msgs` included in

```bash
git clone https://github.com/swri-robotics/gps_umd.git -b ros2-devel
```

When building from source add `COLCON_IGNORE` to `gpsd_client` as
this package is not required and will not build on macOS.

### 2. `sdformat_urdf`

#### 2.1. Library extension

On macOS the `robot_state_publisher` node cannot load the
`sdformat_urdf_plugin` plugin unless the extension is changed:

```bash
cd ./install/sdformat_urdf/lib
ln -s libsdformat_urdf_plugin.so libsdformat_urdf_plugin.dylib
```

#### 2.3. Model URIs

The `sdformat_urdf` plugin requires the `<uri>` element to use
the `package://` prefix for a resource to be located by RViz. At present
this requires the models to be edited.

All occurrences of

    `model://{model_name}`
must be replaced with

    `package://{package_name}/models/{model_name}`


#### 2.4. SDFormat environment variables

The `sdformat_urdf` plugin uses the `sdformat13` libraries to parse the
model description which relies on the environment variable
`SDF_PATH` to resolve model resources. This is usually set in `gz-sim7`,
however when using the plugins standalone, for instance in the bring-up
launch files, `SDF_PATH` must be set otherwise the plugin will not resolve
the models and their dependencies.

```bash
source ~/ros2_ws/install/setup.sh
export SDF_PATH=$GZ_SIM_RESOURCE_PATH
```

This is assigned in the `iris.launch.py` file as `SDF_PATH` is not usually set
by the `ament` environment hooks.
