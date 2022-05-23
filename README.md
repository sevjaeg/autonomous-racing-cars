# Autonomous Racing Cars

## Setup

### ROS Installation

Based on Ubuntu 20.04 LTS (tested with WSL2)

Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu):

```
sudo apt install ros-noetic-desktop-full
```

Source ROS environment by adding to `~/.bashrc`:

```
source /opt/ros/noetic/setup.bash
```

Install dependencies

```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

```
sudo apt-get install ros-noetic-tf2-geometry-msgs ros-noetic-ackermann-msgs ros-noetic-joy ros-noetic-map-server git g++
```

Setup `rosdep`

```
sudo rosdep init
rosdep update
```

Clone this repository

```
git clone https://github.com/sevjaeg/autonomous-racing-cars.git
```

### Simulator Setup

Download modified f1tenth simulator (our changes fix the tf tree to work smoothly with AMCL)

```
cd autonomous-racing-cars
git clone https://github.com/sevjaeg/f1tenth_simulator
```

Create catkin workspace

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

Create symbolic link

```
ln -s <REPO_PATH>/autonomous-racing-cars/f1tenth_simulator
```

Build the simulator package

```
cd ~/catkin_ws
catkin_make
```

Run the setup

```
. ~/catkin_ws/devel/setup.bash
```

Start the simulator with (keyboard mode can be enabled by pressing `k`, then `w`, `a`, `s`, and `d` can be used)

```
roslaunch f1tenth_simulator simulator.launch
```

### Adding New Nodes

Create a symbolic link from the repo to your source
```
cd ~/catkin_ws/src
ln -s <REPO_PATH>/autonomous-racing-cars/<PATH_TO_YOUR_NODE>
```
And build it and don't forget to source everything with `source devel/setup.bash`.


### Debugging Nodes

To be able to debug nodes they need to be built with:
```
catkin_make -DCMAKE_BUILD_TYPE=DEBUG
```

### Google Cartographer

There is a [Setup Guide](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#building-installation) available online. Do not use your standard `catkin_ws` but e.g. `cartographer_ws` to avoid problems with the `catkin_make` command. Besides that follow all steps in the tutorial.

After compiling, the following steps are required for setup:

```
cd ~/catkin_ws/src
ln -s <REPO_PATH>/autonomous-racing-cars/cartographer_config/f110_description/
cp <REPO_PATH>/autonomous-racing-cars/cartographer_config/f110_2d.lua ~/cartographer_ws/install_isolated/share/cartographer_ros/configuration_files
cp <REPO_PATH>/autonomous-racing-cars/cartographer_config/f110_2d.launch ~/cartographer_ws/install_isolated/share/cartographer_ros/launch
```

To run the node, the following command is required

```
source ~/cartographer_ws/install_isolated/setup.bash
```

In a first terminal run the simulator (TODO or the car setup), e.g. with

```
roslaunch wall_follow slam.launch
```

Note that RVIZ might show you some errors due to missing `tf` transformations as the SLAM node is not running yet.

To run the cartographer, open a second terminal and run


and

```
roslaunch cartographer_ros f110_2d.launch
```

Then, the SLAM node should generate a map in the `map_slam` topic. For correct laser scan visualisation you might have to change the fixed frame in rviz to `odom` or `map_slam`.

To save the map, run

```
rosrun map_server map_saver -f <MAP_NAME> map:=/map_slam
```

in a third terminal.

#### Compilation Issue Fixes

Jinja2 version 3.2.X might cause problems

```
pip install jinja2==3.0.0
```

`python` pointing to `python2` might cause problems

```
sudo apt install python-is-python3
```

### AMCL

```
sudo apt-get install ros-noetic-amcl
```

```
roslaunch wall_follow filter.launch
```

### Robot Localisation

```
sudo apt-get install ros-noetic-robot-localization
```

```
roslaunch wall_follow fusion.launch
```

### Planner

```
pip install scikit-image
```

Launch rviz first

```
roslaunch pure_pursuit rviz.launch
```

Then launch the simulator with the planner

```
roslaunch pure_pursuit simulator.launch
```

## Using HW data

Recorded runs from INFHS (lap6 with best quality so far)

```
rosbag play bags/infhs_lap6_left_auto_2022-04-07-13-38-33.bag
```

SLAM output: `infhs_slam_2` (best map based on lap6)

Running AMCL and sensor fusion including visualisation

```
cd lab4
roslaunch filter_fusion_hw.launch
```
