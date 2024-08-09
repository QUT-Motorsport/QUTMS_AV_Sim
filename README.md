# QUTMS_AV_Sim

## Overview

QUTMS_AV_Sim is designed to facilitate development of autonomous systems with little-to-no prior ROS 2 experience. Primarily, it is intended to be used by the Queensland University of Technology Motorsport (QUTMS) team to develop their autonomous vehicle software in ROS 2. However, it is open source and can be used by anyone.
It makes use of the [Gazebo](http://gazebosim.org/) simulator for lightweight ROS 2 specific vehicle URDFs.

QUTMS has and continues to use in varying capacities, forked versions the Formula Student Driverless Simulator (FSDS) and the Edinburgh University Formula Student Simulator (EUFS Sim). Some Gazebos and ROS 2 plugins have also been forked from eufs_sim for this project.

## Installation

### Prerequisites

It is assumed that you have already installed ROS 2 Humble. If you have not, you can follow the instructions [here](https://docs.ros.org/en/humble/Installation.html). QUTMS_Driverless also has environment and workspace installation scripts which can be installed using our member setup guide.

```
cd <YOUR ROS 2 WORKSPACE<> # eg. QUTMS/
git clone https://github.com/QUT-Motorsport/QUTMS_AV_Sim.git
```

If not already installed, install the QUTMS_Driverless repo for our ROS 2 msgs.
```
git clone https://github.com/QUT-Motorsport/QUTMS_Driverless.git
```

### Dependencies

Source existing ROS 2 workspace

```
source install/setup.bash 
# alternatively, if workspace was installed with scripts
a
```

Install ROS 2 dependencies with `rosdep`

```
sudo apt-get update && apt-get upgrade -y
rosdep update
rosdep install -y \
    --rosdistro=${ROS_DISTRO} \
    --ignore-src \
    --from-paths QUTMS_AV_Sim
```

> If this fails due to missing dependency `driverless_msgs`, try building the `driverless_msgs` package first and re-sourcing the workspace.

### Building

Use `colcon build` or build shortcut scripts
```
colcon build --symlink-install --packages-up-to qutms_sim
# alternatively, if workspace was installed with scripts
./build.sh -u qutms_sim
```

## Usage

### Launching the Simulator

```
source install/setup.bash 
# alternatively, if workspace was installed with scripts
a
ros2 launch qutms_sim sim.launch.py
```

### Configuring the Simulator

The simulator can be configured using the config file. This file is located at `QUTMS_AV_Sim/qutms_sim/config/config.yaml`. The file contains a number of parameters, whose effects are documented within.

### Visualising the Simulator

The simulator can be visualised using RViz or Foxglove Studio, these can be configured. With Rviz, you can visualise the vehicle and its sensors. With Foxglove Studio, you can visualise the vehicle, its sensors, and datastreams, in addition to controlling the vehicle and providing a more interactive experience.

With Foxglove Studio, you can download the simulator's `.json` dashboard from github `https://github.com/QUT-Motorsport/QUTMS_AV_Sim/blob/main/qutms_sim/visuals/QUTMS_AV_Sim%20control.json` to load some default visuals.
See the member setup guide for more information on how to use Foxglove Studio and custom dashboards.
