**Table of Contents**

<!-- TOC -->

- [Introduction](#introduction)
- [Dynamic Model](#dynamic-model)
- [Setup the environment](#setup-the-environment)
    - [Install ROS1 Noetic](#install-ros1-noetic)
    - [Setup Crazyflie 2.0 Quadrotor in Gazebo](#setup-crazyflie-20-quadrotor-in-gazebo)
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

# Dynamic Model
The quadrotor model is shown below.

![Drone Frames](/Resources/Photos/Drone%20Frames.png)

Considering two coordinate frames specifically the world coordinate frame - $O_{W}$ and the body coordinate frame $O_{B}$ - the generalized coordinates for a quadrotor model are defined as:

$q = [x\  y\  z\  \phi\  \theta\ \psi]^{T}$

with the translational coordinates $x$, $y$, $z$ with respect to the world frame, and the roll $\phi$, pitch $\theta$ and yaw $\psi$ angles with respect to the body frame.

The control inputs on the system can be considered simply as:

$u = [u_{1}\ u_{2}\ u_{3}\ u_{4}]$

where $u_{1}$ is the force from all the propellers, and $u_{2}$, $u_{3}$, and $u_{4}$ are the moments applied about the body frame axes by the propellers.

For a set of desired control inputs, the desired rotor speeds (i.e. $\omega_{i}$ for $i$ = 1, 2, 3, 4) are obtained by using the “allocation matrix”:

![Formula](/Resources/Photos/Matrix%20Formula.png)

where $k_{F}$ and $k_{M}$ denote the propeller thrust factor and moment factor, respectively.

Considering the generalized coordinates and the control inputs defined above, the simplified equations of motion (assuming small angles) for the translational  accelerations and body frame angular accelerations are derived as:

$\ddot{x} = \frac{1}{m}(cos\phi sin\theta cos\psi\  + \ sin\phi sin\psi)u_{1}$

$\ddot{y} = \frac{1}{m}(cos\phi sin\theta sin\psi\  - \ sin\phi cos\psi)u_{1}$

$\ddot{z} = \frac{1}{m}(cos\phi cos\theta)u_{1}-g$

$\ddot{\phi} =\dot{\theta}\dot{\psi}\frac{I_{y}-I_{z}}{I_{x}}-\frac{I_{p}}{I_{x}}\Omega \dot{\theta}+\frac{1}{I_{x}}u_{2}$

$\ddot{\theta} =\dot{\phi}\dot{\psi}\frac{I_{z}-I_{x}}{I_{y}}+\frac{I_{p}}{I_{y}}\Omega \dot{\phi}+\frac{1}{I_{y}}u_{3}$

$\ddot{\psi} =\dot{\phi}\dot{\theta}\frac{I_{x}-I_{y}}{I_{z}}+\frac{1}{I_{z}}u_{4}$

where $m$ is the quadrotor mass, $g$ is the gravitational acceleration, $I_{p}$ is the propeller moment of inertia, and $I_{x}$, $I_{y}$, $I_{z}$ indicate the quadrotor moment of inertia along the $x$, $y$ and $z$ axes, respectively. Moreover, the term $\Omega$ is expressed as: $\Omega=\omega_{1}-\omega_{2}+\omega_{3}-\omega_{4}$.

The physical parameters for the Crazyflie 2.0 hardware are listed below
|_Parameter_|_Symbol_|_Value_|
|-----------|--------|-------|
|Quadrotor mass|$m$|$27\ g$|
|Quadrotor arm length|$l$|$46\ mm$|
|Quadrotor inertia along x-axis|$I_{x}$|$16.571710*10^{-6}\ kg.m^{2}$|
|Quadrotor inertia along y-axis|$I_{y}$|$16.571710*10^{-6}\ kg.m^{2}$|
|Quadrotor inertia along z-axis|$I_{z}$|$29.261652*10^{-6}\ kg.m^{2}$|
|Propeller moment of Inertia|$I_{p}$|$12.65625*10^{-8}kg.m^{2}$|
|Propeller thrust factor|$k_{F}$|$1.28192*10^{-8}N.s^{2}$|
|Propeller moment factor|$k_{M}$|$5.964552*10^{-3}m$|
|Rotor maximum speed|$\omega_{max}$|$2618\ rad/s$|
|Rotor minimum speed|$\omega_{min}$|$0\ rad/s$|

_Remark 1: As shown in the equations of motion above, the quadrotor system has six DoF, with only four control inputs. As a result, the control of quadrotors is typically done by controlling only the altitude $z$ and the roll-pitch-yaw angles._

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

## Setup Crazyflie 2.0 Quadrotor in Gazebo
To set up the Crazyflie 2.0 quadrotor in Gazebo, we need to install additional ROS dependencies for building packages as below:
```
sudo apt update
sudo apt install ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink
sudo apt install ros-noetic-octomap-mapping ros-noetic-control-toolbox
sudo apt install python3-vcstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev
rosdep update
sudo apt-get install ros-noetic-ros libgoogle-glog-dev
```

We are now ready to create a new ROS workspace and download the ROS packages for the robot:
```
mkdir -p ~/rbe502_project/src
cd ~/rbe502_project/src
catkin_init_workspace # initialize your catkin workspace
cd ~/rbe502_project
catkin init
catkin build -j1
cd ~/rbe502_project/src
git clone -b dev/ros-noetic https://github.com/gsilano/CrazyS.git
git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
```
_Note: a new ROS workspace is needed for the project, because the CrazyS Gazebo package is built using the `catkin build` tool, instead of `catkin_make`._

_Note: -j1 in catkin build is for safety so it does not cause you computer to hang. It makes your code to build on just one core. Slow but ensures it compiles without issues_

We need to build the project workspace using `python_catkin_tools` , therefore we need to configure it:
```
cd ~/rbe502_project
rosdep install --from-paths src -i
rosdep update
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
catkin build -j1
```
_This is gonna take a lot of time. Like a real lot. So maybe make yourself a cup of coffee meanwhile? If you don't like coffee, I don't know your motivation to live :(_

Do not forget to add sourcing to your `.bashrc` file:
```
echo "source ~/rbe502_project/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
With all dependencies ready, we can build the ROS package by the following commands:
```
cd ~/rbe502_project
catkin build -j1
```
To spawn the quadrotor in Gazebo, we can run the following launch file:
```
roslaunch rotors_gazebo crazyflie2_without_controller.launch
```

Congrats, All the setup is done for the simulation to start. You can start writing your own algorithm if you want to!!


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
