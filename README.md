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
The contribution can be also considered as a reference guide for expanding the [F1TENTH simulator](https://github.com/f1tenth-dev/simulator) functionalities by facilitating the integration of the new control algorithms and functionalities.

Some simple case studies are considered to evaluate the performance of the retrieve controller. The code is released under Apache license, thus making it available for scientific and educational activities.

The platform was developed using Ubuntu 18.04 and the Melodic Morenia version of ROS, but it is also fully compatible with Ubuntu 20.04 along with the Noetic Ninjemys distribution of ROS. Although backwards compatibility is guarantee, i.e., the platform is fully compatible with Melodic Morenia version of ROS and Ubuntu 18.04, such configuration is not recommended since the ROS support is expected to be closed in 
