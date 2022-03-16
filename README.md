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
cd autonomous-racing-cars
```

### Simulator Setup

Download f1tenth simulator

```
git clone https://github.com/CPS-TUWien/f1tenth_simulator.git
```

Create catkin workspace

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

Create symbolic link (<YOUR_PATH> is the parent directory of this repository in your file system)

```
ln -s <YOUR_PATH>/autonomous-racing-cars/f1tenth_simulator
```

Build the simulator package

```
cd ~/catkin_ws
catkin_make
```

Start the simulator with (keyboard mode can be enabled by pressing `k`, then `w`, `a`, `s`, and `d` can be used)

```
roslaunch f1tenth_simulator simulator.launch
```
