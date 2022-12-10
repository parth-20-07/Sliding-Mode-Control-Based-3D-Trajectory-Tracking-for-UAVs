**Table of Contents**

<!-- TOC -->

- [Introduction](#introduction)
- [Sliding_mode_controller_for_quadrotor](#sliding_mode_controller_for_quadrotor)
- [How to run the project](#how-to-run-the-project)
- [Interested in Editing the project for your own use?](#interested-in-editing-the-project-for-your-own-use)
    - [Collect the Softwares to setup the project](#collect-the-softwares-to-setup-the-project)
        - [Collect the Project Files](#collect-the-project-files)
        - [Install ROS1 - Noetic](#install-ros1---noetic)
    - [Setup Environment](#setup-environment)
    - [Setup Project](#setup-project)
        - [What each scripts contain?](#what-each-scripts-contain)
- [Designer Details](#designer-details)
- [Acknowledgements](#acknowledgements)
- [License](#license)

<!-- /TOC -->

# Introduction

# Sliding_mode_controller_for_quadrotor

sliding mode controller is designed for altitude and attitude control of the Crazyflie 2.0 to enable the
quadrotor to track desired trajectories and visit a set of desired waypoints.

Desired set of waaypoints are given by :

- p 0 = (0, 0, 0) to p 1 = (0, 0, 1) in 5 seconds
- p 1 = (0, 0, 1) to p 2 = (1, 0, 1) in 15 seconds
- p 2 = (1, 0, 1) to p 3 = (1, 1, 1) in 15 seconds
- p 3 = (1, 1, 1) to p 4 = (0, 1, 1) in 15 seconds
- p 4 = (0, 1, 1) to p 5 = (0, 0, 1) in 15 seconds



# How to run the project


# Interested in Editing the project for your own use?

## Collect the Softwares to setup the project

### Collect the Project Files

### Install ROS1 - Noetic

## Setup Environment

## Setup Project

### What each scripts contain?

# Designer Details

- Designed for:
  - Worcester Polytechnic Institute
  - RBE 502: Robot Control - Final Project
- Designed by:
  - [Parth Patel](mailto:parth.pmech@gmail.com)
  - [Prarthana Sigedar](mailto:)

# Acknowledgements

# License

This project is licensed under [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.en.html) (see [LICENSE.md](LICENSE.md)).

Copyright 2022 Parth Patel Prarthana Sigedar

Licensed under the GNU General Public License, Version 3.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at

_https://www.gnu.org/licenses/gpl-3.0.en.html_

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

<!-- # Documentation TODO

- [X] Introduction
- [X] How to run the project
- [X] Interested in editing the project for your own use?
  - [X] Collect the Softwares to setup the project
    - [X] Collect the Project Files
    - [X] Unity Hub
    - [X] Android Studio
    - [X] Android NDK
    - [X] VS Code
  - [X] Setup Environment
    - [X] Setup Directories
  - [X] Setup Project
    - [X] Launch Project in Unity
    - [X] Import Essential Packages
    - [X] Setup Android Device for App Emulation
    - [X] What each script contains?
    - [X] Export the Android App
- [X] Tools Used
- [X] Designer Details
- [X] Acknowledgements
- [X] License -->
