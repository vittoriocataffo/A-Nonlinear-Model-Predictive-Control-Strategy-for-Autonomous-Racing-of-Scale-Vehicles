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
  preprint = {https://giuseppesilano.net/publications/SMC22.pdf}
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
$ sudo apt install python3-rosdep python3-wstool ros-noetic-ros 
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
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
$ sudo apt install python3 python
$ sudo apt install python-rosinstall python-rosinstall-generator build-essential
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
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

Before you start, you need to install the OpEn framework and its requirements.

1. **Rust**, following the official [installation guide](https://www.rust-lang.org/tools/install),
- Why? The Rust compiler is an essential component of OpEn; you will most likely not need to write (or compile yourself) any Rust code, but OpEn's Python/MATLAB interface will need the compiler to build your optimizer
- How? Follow the [instructions](https://www.rust-lang.org/tools/install); in a nutshell, on Linux and MacOSX run:

```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

and add `/.cargo/bin` to your path - e.g., on Linux, put the following line in your `~/.profile` file:

```
# Add this to your ~/.profile file
export PATH="$HOME/.cargo/bin:$PATH"
```

then **logout and login again** (or restart) for this to take effect.

- **clang**, following this [guide](https://github.com/rust-lang/rust-bindgen/blob/master/book/src/requirements.md). Why? OpEn uses CasADi to build certain functions in C, which then need to be called from OpEn's core solver in Rust. For that purpose we need **bindgen**, which requires **clang**.

2. Python Interface

As simple as `pip install opengen`. You might need to prepend `sudo` on some Linux systems. Note that OpEn requires Python **version 3.5 or newer**. You might, therefore, need to install it using `pip3 install opengen`.

> **Note** OpEn may run on earlier versions of Python (as old as 2.7), but we cannot promise you that (the main difficulty being the installation of dependencies). In that case, it is strongly recommend that you use `virtualenv`. To install OpEn in a virtual environment, using `virtualenv`, you first need to create such an environment, then activate it, and lastly, install `opengen` as above using `pip`. That is, you need to run:
>
>```
>virtualenv -p python3.6 venv36
>source venv36/bin/activate
>pip install opengen
>```

3. Install opengen

Go into `optimization-engine/open-codegen` and create a virtual environment:

```
cd optimization-engine/open-codegen
virtualenv -p python3.6 venvopen
source venvopen/bin/activate
python setup.py install
```

You're ready to go! It's a good idea to use an IDE, such as [PyCharm](https://www.jetbrains.com/pycharm/). Use the above virtual environment (`venvopen`) in PyCharm:

- go to Run > Edit Configurations > Add new configuration
- Script path: specify `main.py`
- Working dir: `optimization-engine/open-codegen/opengen`
- Python interpreter: `venvopen`

Install OpEn in Rust is as easy as:

```
cd optimization-engine
cargo build
```

If you need to use `opengen` - the Python interface of OpEn - with a local version of the Rust library, use `with_open_version(local_path=...)` in your code. Read the [advanced options](https://alphaville.github.io/optimization-engine/docs/python-advanced#build-options) for details.

At the end, you need to add the following lines in your `.bashrc` file

```
# Install Ruby Gems to ~/gems
export GEM_HOME="$HOME/gems"
export PATH="$HOME/gems/bin:$PATH"

# Add this to your ~/.profile file
export PATH="$HOME/.cargo/bin:$PATH"
. "$HOME/.cargo/env"
```

Further info can be found at the [link](https://alphaville.github.io/optimization-engine/docs/installation).

Basic Usage
============

Launching the simulation is quite simple, so as customizing it. However, some actions should be performed before running the scripts, such as building the NMPC controller.

```
$ cd ~
$ mkdir -p autosim_nmpc
$ git clone -b nmpc_panoc https://github.com/vittoriocataffo/A-Nonlinear-Model-Predictive-Control-Strategy-for-Autonomous-Racing-of-Scale-Vehicles.git
$ cd PANOC_DYNAMIC_MOTOR_MODEL
$ python3 all_wheel_drive_PANOC_DINAMYC_motor_model.py
$ python3 main_loop_DYNAMIC_motor.py
$ pythone3 PANOC_DINAMYC_motor_model.py
$ cd ../OBS_PANOC_DYNAMIC_MOTOR_MODEL
$ python3 OBS_all_wheel_drive_PANOC_DINAMYC_motor_model.py
$ python3 OBS_main_loop_DYNAMIC_motor.py
$ python3 OBS_PANOC_DINAMYC_motor_model.py
```

The result will be a folder entitled `dynamic_my_optimizer` containing the build of the NMPC. To make everything work, it is fundamental to correct link the generated file with the scripts running of controller running along with the simulation. Therefore, att the end of the building process, you need to follow the instruction reported at terminal line. In other words,

1. Edit the path provided as argument to the function `sys.path.insert` in the files entitled `OBS_all_wheel_drive_NEW_MOTOR_PANOC_my_mpc_node_DYNAMIC.py`, `NEW_MOTOR_PANOC_my_mpc_node_DYNAMIC_OBS.py`, `NEW_MOTOR_PANOC_my_mpc_node_DYNAMIC.py`and `all_wheel_drive_NEW_MOTOR_PANOC_my_mpc_node_DYNAMIC.py` contained in the `autosim_ws/src/simulator/scripts` folder. This will connect the built NMPC controller with the scripts running the simulation.

2. Import the maps of the scenarios contained in the `Map_track1`, `Map_track2` and `Map_track3` folders. To do that, you need to edit the path of the function `csv_file = np.genfromtxt` with that where the `autosim_nmpc` folder is located. 

3. Finally, you need to edit the path provided as argument of the function `f = open`. This can be located everywhere on your machine. This will contain the csv files saved by running the Gazebo simulation. In case you are going to save the files in a folder, do not forget to create the folder (`mkdir -p path/NAME_FOLDER`) before adding to the Python script.

> **Note**. In case you should get back with the error `ERROR: cannot launch node of type [f1tenth-sim/set_racecar_state.py]: Cannot locate node of type [set_racecar_state.py] in package [f1tenth-sim]. Make sure file exists in package path and permission is set to executable (chmod +x)`, you need to give the right permissions to the Python scripts by runnin the command 
> 
>  ```
>  $ cd ~/autosim_ws/src/simulator/src 
>  $ chmod +x set_racecar_state.py
>  ```

> **Note**. A nice feature to have before running the simulation is to replace the standard Gazebo texture for the asphalt with that provided along with the repository. To do that, in case of Ubuntu 20.04 with Gazebo 11, simply execute the commands 
>
>```
>$ cd ~/autosim/gazebo
>$ sudo cp road1.jpg /usr/share/gazebo-11/media/materials/textures
>```
>
>while in case of Ubuntu 18.04 with Gazebo 9
>
>```
>$ cd ~/autosim/gazebo
>$ sudo cp road1.jpg /usr/share/gazebo-9/media/materials/textures
>```

Now, you can run in a terminal the command:

```
$ roslaunch f1tenth-sim NEW_track_3.master
```
and in a seperate terminal window

```
$ rosrun f1tenth-sim all_wheel_drive_NEW_MOTOR_PANOC_my_mpc_node_DYNAMIC.py
```

To run a simulation considering obstacles placed along the track:

```
$ roslaunch f1tenth-sim NEW_track_3_OBS.master
```

and in a separate terminal window

```
$ rosrun f1tenth-sim OBS_all_wheel_drive_NEW_MOTOR_PANOC_my_mpc_node_DYNAMIC.py
```

You may decide to modify the parameters used by the controller to simulate the vehicle dynamics within the time horizon. Below is a legend with the available Python scripts and the parameters used.

- `NEW_MOTOR_PANOC_my_mpc_node_DYNAMIC.py` It considers the model parameters as described in [the paper](https://www.sciencedirect.com/science/article/pii/S2405896320322722).
- `NEW_MOTOR_PANOC_my_mpc_node_DYNAMIC_OBS.py` It considers the model parameters as described in [the paper](https://www.sciencedirect.com/science/article/pii/S2405896320322722).
- `all_wheel_drive_NEW_MOTOR_PANOC_my_mpc_node_DYNAMIC.py` It considers the identified model parameters as described in [the paper](https://github.com/vittoriocataffo/A-Nonlinear-Model-Predictive-Control-Strategy-for-Autonomous-Racing-of-Scale-Vehicles/wiki/Publications).
- `OBS_all_wheel_drive_NEW_MOTOR_PANOC_my_mpc_node_DYNAMIC.py`. It considers the identified model parameters as described in [the paper](https://github.com/vittoriocataffo/A-Nonlinear-Model-Predictive-Control-Strategy-for-Autonomous-Racing-of-Scale-Vehicles/wiki/Publications).

Bugs & Feature Requests
========================

Please report bugs and request features by using the [Issue Tracker](https://github.com/vittoriocataffo/A-Nonlinear-Model-Predictive-Control-Strategy-for-Autonomous-Racing-of-Scale-Vehicles/issues). Furthermore, please see the [Contributing.md](https://github.com/vittoriocataffo/A-Nonlinear-Model-Predictive-Control-Strategy-for-Autonomous-Racing-of-Scale-Vehicles/blob/main/CONTRIBUTING.md) file if you plan to help us to improve the repository's features.

YouTube videos
===============

In this section a video providing the effectiveness of the platform and how it works is reported. Further videos can be found in the related YouTube channel. Have fun! :)

[![NMPC strategy for Autonomous Racing of Scale Vehicles](https://github.com/vittoriocataffo/A-Nonlinear-Model-Predictive-Control-Strategy-for-Autonomous-Racing-of-Scale-Vehicles/wiki/img/cover_SMC22.png)](https://youtu.be/w5c328rQmX4 "NMPC strategy for Autonomous Racing of Scale Vehicles")


