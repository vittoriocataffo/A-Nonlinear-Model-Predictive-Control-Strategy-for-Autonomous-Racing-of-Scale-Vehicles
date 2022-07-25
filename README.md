[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg?style=flat-square)](http://makeapullrequest.com)
[![first-timers-only](https://img.shields.io/badge/first--timers--only-friendly-blue.svg?style=flat-square)](https://www.firsttimersonly.com/)
[![All Contributors](https://img.shields.io/badge/all_contributors-2-orange.svg?style=flat-square)](#contributors)

NMPC strategy for Autonomous Racing of Scale Vehicles
=======================================================

This repository is an extension of the [F1TENTH simulator](https://github.com/f1tenth-dev/simulator). Such a simulator mirrors the behavior of an autonomous racing scale vehicle. The repository is endowed with the necessary files to run the Nonlinear Model Predictive Control (NMPC) strategy described in the paper whose references are given below

```console
@INPROCEEDINGS{Silano2018MED,
  author = {Cataffo, Vittorio and Silano, Giuseppe and Iannelli, Luigi and Puig, Vicenç and Glielmo, Luigi},
  booktitle = {2022 IEEE International Conference on Systems, Man and Cybernetics (SMC)}},
  title = {{A Nonlinear Model Predictive Control Strategy for Autonomous Racing of Scale Vehicles}},
  year = {2022},
  pages = {--},
  doi = {},
  issn = {},
  month = October,
  link = {},
  preprint = {}
}
```
If you are using this simulator for research purposes especially for your publication, please take a look at the [Publications page](https://github.com/vittoriocataffo/A-Nonlinear-Model-Predictive-Control-Strategy-for-Autonomous-Racing-of-Scale-Vehicles/wiki/Publications). 

The contribution can be also considered as a reference guide for expanding the [F1TENTH simulator](https://github.com/f1tenth-dev/simulator) functionalities by facilitating the integration of the new control algorithms and functionalities.

Some simple case studies are considered to evaluate the performance of the retrieve controller. Further instruction on how to execute them are reported in the following. The code is released under Apache license, thus making it available for scientific and educational activities.

The platform was developed using Ubuntu 18.04 and the Melodic Morenia version of ROS, but it is also fully compatible with Ubuntu 20.04 along with the Noetic Ninjemys distribution of ROS. Although backwards compatibility is guarantee, i.e., the platform is fully compatible with Melodic Morenia version of ROS and Ubuntu 18.04, such configuration is not recommended since the ROS support is expected to be closed in April 2023.

Below there are the instructions necessary for getting started. 

Installation Instructions - Ubuntu 20.04 with ROS Noetic and Gazebo 11
========================================================================

To use the code developed and stored in this repository some preliminary actions are needed. They are listed below.

1. Install and initialize ROS Melodic desktop full, additional ROS packages, catkin-tools, and wstool:

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt install curl # if you haven't already installed curl
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo apt update
$ sudo apt install ros-noetic-desktop-full ros-noetic-joy ros-noetic-control-toolbox
$ sudo apt install python3-vcstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev
$ sudo apt install python3 python
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt-get install python3-rosdep python3-wstool ros-noetic-ros libgoogle-glog-dev
```

2. Install all packages for Simultaneous Localization and Mapping (SLAM) and F1TENTH preliminaries

```
$ sudo apt-get install ros-noetic-navigation ros-noetic-teb-local-planner* ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control ros-noetic-ackermann-msgs ros-noetic-serial qt5-default
```

3. If you don't have ROS workspace yet you can do so by

```
$ mkdir -p ~/autosim_ws/src
$ cd ~/autosim_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ cd ~/autosim_ws/
$ catkin init
$ cd ~/autosim_ws/src
$ mkdir -p simulator
$ git clone https://github.com/vittoriocataffo/A-Nonlinear-Model-Predictive-Control-Strategy-for-Autonomous-Racing-of-Scale-Vehicles.git
$ cd ~/autosim_ws/src
$ git clone https://github.com/mit-racecar/particle_filter.git
$ git clone https://github.com/kctess5/range_libc.git
$ https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
$ cd ~/autosim_ws
```

4. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

```
$ rosdep install --from-paths src -i
$ rosdep update
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
$ catkin build
```

5. Add sourcing to your `.bashrc` file

```
$ echo "source ~/autosim_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

Installation Instructions - Ubuntu 18.04 with ROS Melodic and Gazebo 9
========================================================================

To use the code developed and stored in this repository some preliminary actions are needed. They are listed below.

1. Install and initialize ROS Melodic desktop full, additional ROS packages, catkin-tools, and wstool:

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
$ sudo apt install ros-melodic-desktop-full ros-melodic-joy ros-melodic-control-toolbox
$ sudo apt install python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ sudo apt install python-rosinstall python-rosinstall-generator build-essential

```

2. Install all packages for Simultaneous Localization and Mapping (SLAM) and F1TENTH preliminaries

```
$ sudo apt-get install ros-melodic-navigation ros-melodic-teb-local-planner* ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros-control ros-melodic-ackermann-msgs ros-melodic-serial qt4-default
```

3. If you don't have ROS workspace yet you can do so by

```
$ mkdir -p ~/autosim_ws/src
$ cd ~/autosim_ws/src
$ catkin_init_workspace  # initialize your catkin workspace
$ cd ~/autosim_ws/
$ catkin init
$ cd ~/autosim_ws/src
$ mkdir -p simulator
$ git clone https://github.com/vittoriocataffo/A-Nonlinear-Model-Predictive-Control-Strategy-for-Autonomous-Racing-of-Scale-Vehicles.git
$ cd ~/autosim_ws/src
$ git clone https://github.com/mit-racecar/particle_filter.git
$ git clone https://github.com/kctess5/range_libc.git
$ https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
$ cd ~/autosim_ws
```

4. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

```
$ rosdep install --from-paths src -i
$ rosdep update
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
$ catkin build
```

5. Add sourcing to your `.bashrc` file

```
$ echo "source ~/autosim_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

OpEn Framework
===============

Basic Usage
============

Bugs & Feature Requests
========================

Please report bugs and request features by using the [Issue Tracker](https://github.com/vittoriocataffo/A-Nonlinear-Model-Predictive-Control-Strategy-for-Autonomous-Racing-of-Scale-Vehicles/issues). Furthermore, please see the [Contributing.md](https://github.com/vittoriocataffo/A-Nonlinear-Model-Predictive-Control-Strategy-for-Autonomous-Racing-of-Scale-Vehicles/blob/main/CONTRIBUTING.md) file if you plan to help us to improve the repository's features.

YouTube videos
===============

In this section a video providing the effectiveness of the platform and how it works is reported. Further videos can be found in the related YouTube channel. Have fun! :)

[![NMPC strategy for Autonomous Racing of Scale Vehicles](https://github.com/vittoriocataffo/A-Nonlinear-Model-Predictive-Control-Strategy-for-Autonomous-Racing-of-Scale-Vehicles/wiki/img/cover_SMC22.png)](https://youtu.be/w5c328rQmX4 "NMPC strategy for Autonomous Racing of Scale Vehicles")


