**Table of Contents**

<!-- TOC -->

- [Introduction](#introduction)
- [Setup the environment](#setup-the-environment)
    - [Install ROS1 Noetic](#install-ros1-noetic)
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
The objective of this project is to develop a robust control scheme to enable a quadrotor to track desired trajectories in the presence of external disturbances.

The control design under study will be tested on the Crazyflie 2.0 platform. Crazyflie is a quadrotor that is classified as a micro air vehicle (MAV), as it only weighs 27 grams and can fit in your hand. The size makes it ideal for flying inside a lab without trashing half the interior. Even though the propellers spin at high RPMs, they are soft and the torque in the motors is very low when compared to a brushless motor, making it relatively crash tolerant. The Crazyflie 2.0 features four 7mm coreless DC-motors that give the Crazyflie a maximum takeoff weight of 42g.


The Crazyflie 2.0 is an open source project, with source code and hardware design both documented
and available. For more information, see the link below:
[https://www.bitcraze.io/products/old-products/crazyflie-2-0/](https://www.bitcraze.io/products/old-products/crazyflie-2-0/)

Crazyflie 2.0 Quadrotor Hardware

![Hardware](/Resources/Photos/CrazyFlie.png)

Crazyflie 2.0 Quadrotor Gazebo Simulation

![Drone Model](/Resources/Photos/Gazebo.png)

_Note: The Complete project is build and tested on Ubuntu 20.04LTS with ROS Noetic and not tested on any other system. Kindly proceed with caution and loads of google usage for porting it to any other system. And as always, common sense is must before anything!!_

**Sliding_mode_controller_for_quadrotor**

sliding mode controller is designed for altitude and attitude control of the Crazyflie 2.0 to enable the
quadrotor to track desired trajectories and visit a set of desired waypoints.

Desired set of waaypoints are given by :

- p 0 = (0, 0, 0) to p 1 = (0, 0, 1) in 5 seconds
- p 1 = (0, 0, 1) to p 2 = (1, 0, 1) in 15 seconds
- p 2 = (1, 0, 1) to p 3 = (1, 1, 1) in 15 seconds
- p 3 = (1, 1, 1) to p 4 = (0, 1, 1) in 15 seconds
- p 4 = (0, 1, 1) to p 5 = (0, 0, 1) in 15 seconds

# Setup the environment

## Install ROS1 Noetic

1. Setup your sources.list
    Setup your computer to accept software from packages.ros.org.
    ```
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```
2. Set up your keys
    ```
    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    ```
3. Installation
    First, make sure your Debian package index is up-to-date:
    ```
    sudo apt update
    ```
    Desktop-Full Install: (Recommended) : Everything in Desktop plus 2D/3D simulators and 2D/3D perception packages
    ```
    sudo apt install ros-noetic-desktop-full
    ```
4. Environment setup
    You must source this script in every bash terminal you use ROS in.
    ```
    source /opt/ros/noetic/setup.bash
    ```
    It can be convenient to automatically source this script every time a new shell is launched. These commands will do that for you.
    ```
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
5. Dependencies for building packages
    Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. For example, [rosinstall](https://wiki.ros.org/rosinstall) is a frequently used command-line tool that enables you to easily download many source trees for ROS packages with one command.
    
    To install this tool and other dependencies for building ROS packages, run:
    ```
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    ```
    Initialize rosdep
    
    Before you can use many ROS tools, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS. If you have not yet installed rosdep, do so as follows.
    ```
    sudo apt install python3-rosdep
    ```
    With the following, you can initialize rosdep.
    ```
    sudo rosdep init
    rosdep update
    ```
6. Create a workspace
    ```
    mkdir -p ~/rbe502_ros/src
    cd ~/rbe502_ros
    catkin_make
    echo "source ~/rbe502_ros/devel/setup.bash" >> ~/.bashrc
    source ~/rbe502_ros/devel/setup.bash
    ```
7. Install the Control and Effort Dependencies for Gazebo
    ```
    sudo apt-get update
    sudo apt-get upgrade
    sudo apt update
    sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
    sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
    ```
8. With all dependencies ready, we can build the ROS package by the following commands.
    ```
    cd ~/rbe502_ros
    catkin_make
    ```


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
