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

> TODO

[Setup Guide](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html#building-installation)

### AMCL

> TODO

```
sudo apt-get install ros-noetic-amcl
```

### Robot Pose EKF

This package is used to fuse the AMCL output with the IMU data. Install it with

```
sudo apt-get install ros-noetic-robot-pose-ekf
```
