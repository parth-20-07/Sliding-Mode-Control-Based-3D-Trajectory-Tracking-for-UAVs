**Table of Contents**

<!-- TOC -->

- [Introduction](#introduction)
- [Dynamic Model](#dynamic-model)
- [Setup the environment](#setup-the-environment)
    - [Install ROS1 Noetic](#install-ros1-noetic)
    - [Setup Crazyflie 2.0 Quadrotor in Gazebo](#setup-crazyflie-20-quadrotor-in-gazebo)
- [Problem Statement](#problem-statement)
    - [Part 1: Trajectory Generation](#part-1-trajectory-generation)
    - [Part 2: Controller Design](#part-2-controller-design)
- [What each scripts contain?](#what-each-scripts-contain)
- [Packages used](#packages-used)
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

# Dynamic Model
The quadrotor model is shown below.

![Drone Frames](/Resources/Photos/Drone%20Frames.png)

Considering two coordinate frames specifically the world coordinate frame - $O_{W}$ and the body coordinate frame $O_{B}$ - the generalized coordinates for a quadrotor model are defined as:

$$
\begin{equation}
q = [x\  y\  z\  \phi\  \theta\ \psi]^{T}\notag\\
\end{equation}
$$

with the translational coordinates $x$, $y$, $z$ with respect to the world frame, and the roll $\phi$, pitch $\theta$ and yaw $\psi$ angles with respect to the body frame.

The control inputs on the system can be considered simply as:

$$
\begin{equation}
u = [u_{1}\ u_{2}\ u_{3}\ u_{4}]\notag\\
\end{equation}
$$

where $u_{1}$ is the force from all the propellers, and $u_{2}$, $u_{3}$, and $u_{4}$ are the moments applied about the body frame axes by the propellers.

For a set of desired control inputs, the desired rotor speeds (i.e. $\omega_{i}$ for $i$ = 1, 2, 3, 4) are obtained by using the “allocation matrix”:

```math
\begin{bmatrix}
\omega_{1}^2\\
\omega_{2}^2\\
\omega_{3}^2\\
\omega_{4}^2
\end{bmatrix}

=

\begin{bmatrix}

\frac{1}{4k_{f}} & -\frac{\sqrt2}{4k_{f}l} & -\frac{\sqrt2}{4k_{f}l} & -\frac{1}{4k_{f}k_{m}}\\
\frac{1}{4k_{f}} & -\frac{\sqrt2}{4k_{f}l} & \frac{\sqrt2}{4k_{f}l} & \frac{1}{4k_{f}k_{m}}\\
\frac{1}{4k_{f}} & \frac{\sqrt2}{4k_{f}l} & \frac{\sqrt2}{4k_{f}l} & -\frac{1}{4k_{f}k_{m}}\\
\frac{1}{4k_{f}} & \frac{\sqrt2}{4k_{f}l} & -\frac{\sqrt2}{4k_{f}l} & \frac{1}{4k_{f}k_{m}}
\end{bmatrix}

\begin{bmatrix}

u_{1}\\
u_{2}\\
u_{3}\\
u_{4}
\end{bmatrix}
```

where $k_{F}$ and $k_{M}$ denote the propeller thrust factor and moment factor, respectively.

Considering the generalized coordinates and the control inputs defined above, the simplified equations of motion (assuming small angles) for the translational  accelerations and body frame angular accelerations are derived as:

$$
\begin{equation}
\ddot{x} = \frac{1}{m}(cos\phi sin\theta cos\psi\  + \ sin\phi sin\psi)u_{1}\notag
\end{equation}
$$

$$
\begin{equation}
\ddot{y} = \frac{1}{m}(cos\phi sin\theta sin\psi\  - \ sin\phi cos\psi)u_{1}\notag
\end{equation}
$$

$$
\begin{equation}
\ddot{z} = \frac{1}{m}(cos\phi cos\theta)u_{1}-g\notag
\end{equation}
$$

$$
\begin{equation}
\ddot{\phi} =\dot{\theta}\dot{\psi}\frac{I_{y}-I_{z}}{I_{x}}-\frac{I_{p}}{I_{x}}\Omega \dot{\theta}+\frac{1}{I_{x}}u_{2}\notag
\end{equation}
$$

$$
\begin{equation}
\ddot{\theta} =\dot{\phi}\dot{\psi}\frac{I_{z}-I_{x}}{I_{y}}+\frac{I_{p}}{I_{y}}\Omega \dot{\phi}+\frac{1}{I_{y}}u_{3}\notag
\end{equation}
$$

$$
\begin{equation}
\ddot{\psi} =\dot{\phi}\dot{\theta}\frac{I_{x}-I_{y}}{I_{z}}+\frac{1}{I_{z}}u_{4}\notag
\end{equation}
$$

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

Remark 1: As shown in the equations of motion above, the quadrotor system has six DoF, with only four control inputs. As a result, the control of quadrotors is typically done by controlling only the altitude $z$ and the roll-pitch-yaw angles.

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
_This is gonna take a lot of time. Like a real lot. So maybe make yourself a cup of coffee meanwhile? If you don't like coffee, I don't know your motivation to live :worried:_

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

Congrats, All the setup is done for the simulation to start. You can start writing your own algorithm if you want to!! :smiley:

# Problem Statement
Design a sliding mode controller for _altitude_ and _attitude_ control of the Crazyflie 2.0 to enable the quadrotor to track desired trajectories and visit a set of desired waypoints. 

The main components of the project are described below.

## Part 1: Trajectory Generation

Write a MATLAB or Python script to generate quintic (fifth-order) trajectories (position, velocity and acceleration) for the translational coordinates $(x, y, z)$ of Crazyflie. The quadrotor is supposed to start from the origin $p_{0} = (0, 0, 0)$ and visit five waypoints in sequence. The waypoints to visit are:
- $p_{0} = (0, 0, 0)$ to $p_{1} = (0, 0, 1)$ in 5 seconds
- $p_{1} = (0, 0, 1)$ to $p_{2} = (1, 0, 1)$ in 15 seconds
- $p_{2} = (1, 0, 1)$ to $p_{3} = (1, 1, 1)$ in 15 seconds
- $p_{3} = (1, 1, 1)$ to $p_{4} = (0, 1, 1)$ in 15 seconds
- $p_{4} = (0, 1, 1)$ to $p_{5} = (0, 0, 1)$ in 15 seconds

The sequence of visiting the waypoints does matter. The velocity and acceleration at each waypoint must be equal to zero.

**Solution:**

A quintic (fifth-order) equation is used for trajectory solution.

$$
\begin{equation}
q = a_{0} + a_{1}t + a_{2}t^2 + a_{3}t^3 + a_{4}t^4 + a_{5}t^5\notag
\end{equation}
$$

$$
\begin{equation}
\dot{q} = a_{1} + 2a_{2}t + 3a_{3}t^2 + 4a_{4}t^3 + 5a_{5}t^4\notag
\end{equation}
$$

$$
\begin{equation}
\ddot{q} = 2a_{2} + 6a_{3}t + 12a_{4}t^2 + 20a_{5}t^3\notag
\end{equation}
$$

The $t_{0}$ and $t_{f}$ for each of the position values are provided. Considering $v_{0}$, $v_{f}$, $a_{0}$, $a_{f}$ are zero. Placing the values for $t_{0}$ amd $t_{f}$ along with the corresponding coordinates, the resulting equation results as:

```math
\begin{bmatrix}
1 & t_{0} & t_{0}^2 & t_{0}^3 & t_{0}^4 & t_{0}^5\\
0 & 1 & 2t_{0} & 3t_{0}^2 & 4t_{0}^3 & 5t_{0}^4\\
0 & 0 & 2 & 6t_{0} & 12t_{0}^2 & 20t_{0}^3\\
1 & t_{f} & t_{f}^2 & t_{f}^3 & t_{f}^4 & t_{f}^5\\
0 & 1 & 2t_{f} & 3t_{f}^2 & 4t_{f}^3 & 5t_{f}^4\\
0 & 0 & 2 & 6t_{f} & 12t_{f}^2 & 20t_{f}^3
\end{bmatrix}

\begin{bmatrix}
a_{0}\\
a_{1}\\
a_{2}\\
a_{3}\\
a_{4}\\
a_{5}
\end{bmatrix}

=

\begin{bmatrix}
q_{0}\\
\dot{q_{0}}\\
\ddot{q_{0}}\\
q_{f}\\
\dot{q_{f}}\\
\ddot{q_{f}}\\
\end{bmatrix}
```

Thus, the coefficients can be calculated using:

```math
\begin{bmatrix}
a_{0}\\
a_{1}\\
a_{2}\\
a_{3}\\
a_{4}\\
a_{5}
\end{bmatrix}

=
\begin{bmatrix}
1 & t_{0} & t_{0}^2 & t_{0}^3 & t_{0}^4 & t_{0}^5\\
0 & 1 & 2t_{0} & 3t_{0}^2 & 4t_{0}^3 & 5t_{0}^4\\
0 & 0 & 2 & 6t_{0} & 12t_{0}^2 & 20t_{0}^3\\
1 & t_{f} & t_{f}^2 & t_{f}^3 & t_{f}^4 & t_{f}^5\\
0 & 1 & 2t_{f} & 3t_{f}^2 & 4t_{f}^3 & 5t_{f}^4\\
0 & 0 & 2 & 6t_{f} & 12t_{f}^2 & 20t_{f}^3
\end{bmatrix}^{-1}
\begin{bmatrix}
q_{0}\\
\dot{q_{0}}\\
\ddot{q_{0}}\\
q_{f}\\
\dot{q_{f}}\\
\ddot{q_{f}}
\end{bmatrix}
```


Using this, the resulting equations of motion are:

Trajectory 1:

```math
p_{1d}(t) = 
\begin{bmatrix}
0 & 0 & 0.0800t^3-0.0240t^4+0.0019t^5\notag
\end{bmatrix}
```

```math
v_{1d}(t) = 
\begin{bmatrix}
0 & 0 & 0.2400t^2-0.0960t^3+0.0096t^4\notag
\end{bmatrix}
```

```math
a_{1d}(t) = 
\begin{bmatrix}
0 & 0 & 0.4800t-0.2880t^2+0.0384t^4\notag
\end{bmatrix}
```

Trajectory 2:

```math
p_{2d}(t) = 
\begin{bmatrix}
0.0030t^3-2.9630*10^{-4}t^4+7.9012*10^{-6}t^5 & 0 & 1\notag
\end{bmatrix}
```

```math
v_{2d}(t) = 
\begin{bmatrix}
0.0089t^2-0.0012t^3+3.9506*10^{-5}t^4 & 0 & 0\notag
\end{bmatrix}
```

```math
a_{2d}(t) = 
\begin{bmatrix}
0.0178t - 0.0036t^2+1.5802*10^{-4}t^3 & 0 & 0\notag
\end{bmatrix}
```

Trajectory 3:


```math
p_{3d}(t) = 
\begin{bmatrix}
1 & 0.0030t^3-2.9630*10^{-4}t^4+7.9012*10^{-6}t^5 & 1\notag
\end{bmatrix}
```

```math
v_{3d}(t) = 
\begin{bmatrix}
0 & 0.0089t^2-0.0012t^3+3.9506*10^{-5}t^4 & 0\notag
\end{bmatrix}
```

```math
a_{3d}(t) = 
\begin{bmatrix}
0 & 0.0178t-0.0036t^2+1.5802*10^{-4}t^3 & 0\notag
\end{bmatrix}
```

Trajectory 4:


```math
p_{4d}(t) = 
\begin{bmatrix}
1- 0.0030t^3+2.9630*10^{-4}t^4-7.9012*10^{-6}t^5 & 1 & 1\notag
\end{bmatrix}
```

```math
v_{4d}(t) = 
\begin{bmatrix}
-0.0089t^2+0.0012t^3-3.9506*10^{-5}t^4 & 0 & 0\notag
\end{bmatrix}
```

```math
a_{4d}(t) = 
\begin{bmatrix}
- 0.0178t+0.0036t^2-1.5802*10^{-4}t^3 & 0 & 0\notag
\end{bmatrix}
```

Trajectory 5:


```math
p_{5d}(t) = 
\begin{bmatrix}
0 & 1 - 0.0030t^3+2.9630*10^{-4}t^4-7.9012*10^{-6}t^5 & 1\notag
\end{bmatrix}
```

```math
v_{5d}(t) = 
\begin{bmatrix}
0 & - 0.0089t^2+0.0012t^3-3.9506*10^{-5}t^4 & 0\notag
\end{bmatrix}
```

```math
a_{5d}(t) = 
\begin{bmatrix}
0 & - 0.0178t+0.0036t^2-1.5802*10^{-4}t^3 & 0\notag
\end{bmatrix}
```

The resultant trajectories will look as shown:

![Desired Trajectories](/Resources//Photos/des_traj.png)

**Code:**

The complete trajectory coordinates are generated by calling the `traj_evaluate()` function. It takes the `self.t` as input for the current time and based of some conditional statements to evaulate the correct trajectories with respect to current time, it returns the $x_{d}, y_{d}, z_{d}, \dot{x_{d}}, \dot{y_{d}}, \dot{z_{d}}, \ddot{x_{d}}, \ddot{y_{d}}$ and $\ddot{z_{d}}$ coordinates.

```
def traj_evaluate(self):
    # Switching the desired trajectory according to time
    if self.t <= 70:
        if self.t <= 5:
            t0, tf, x0, y0, z0, xf, yf, zf, v0, vf, ac0, acf, t = (
                0, 5, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, self.t)
            print(f'Lift OFF - T:{round(self.t,3)}')
        elif self.t <= 20:
            t0, tf, x0, y0, z0, xf, yf, zf, v0, vf, ac0, acf, t = (
                5, 20, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, self.t)
            print(f'Path 1 - T:{round(self.t,3)}')
        elif self.t <= 35:
            t0, tf, x0, y0, z0, xf, yf, zf, v0, vf, ac0, acf, t = (
                20, 35, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, self.t)
            print(f'Path 2 - T:{round(self.t,3)}')
        elif self.t <= 50:
            t0, tf, x0, y0, z0, xf, yf, zf, v0, vf, ac0, acf, t = (
                35, 50, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, self.t)
            print(f'Path 3 - T:{round(self.t,3)}')
        elif self.t <= 65:
            t0, tf, x0, y0, z0, xf, yf, zf, v0, vf, ac0, acf, t = (
                50, 65, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, self.t)
            print(f'Path 4 - T:{round(self.t,3)}')
        else:
            t0, tf, x0, y0, z0, xf, yf, zf, v0, vf, ac0, acf, t = (
                65, 70, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, self.t)
            print(f'Landing - T:{round(self.t,3)}')
        A = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5],
                      [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
                      [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
                      [1, tf, tf**2, tf ** 3, tf ** 4, tf ** 5],
                      [0, 1, 2*tf, 3 * tf ** 2, 4 * tf ** 3, 5 * tf ** 4],
                      [0, 0, 2, 6 * tf, 12 * tf ** 2, 20 * tf ** 3]])
        bx = np.array(([x0], [v0], [ac0], [xf], [vf], [acf]))
        by = np.array(([y0], [v0], [ac0], [yf], [vf], [acf]))
        bz = np.array(([z0], [v0], [ac0], [zf], [vf], [acf]))

        ax = np.dot(np.linalg.inv(A), bx)
        ay = np.dot(np.linalg.inv(A), by)
        az = np.dot(np.linalg.inv(A), bz)

        # Position
        xd = ax[0] + ax[1]*t + ax[2]*t**2 + \
            ax[3]*t**3 + ax[4]*t**4 + ax[5]*t**5
        yd = ay[0] + ay[1]*t + ay[2]*t**2 + \
            ay[3]*t**3 + ay[4]*t**4 + ay[5]*t**5
        zd = az[0] + az[1]*t + az[2]*t**2 + \
            az[3]*t**3 + az[4]*t**4 + az[5]*t**5

        # Velocity
        xd_dot = ax[1] + 2*ax[2]*t + 3*ax[3] * \
            t**2 + 4*ax[4]*t**3 + 5*ax[5]*t**4
        yd_dot = ay[1] + 2*ay[2]*t + 3*ay[3] * \
            t**2 + 4*ay[4]*t**3 + 5*ay[5]*t**4
        zd_dot = az[1] + 2*az[2]*t + 3*az[3] * \
            t**2 + 4*az[4]*t**3 + 5*az[5]*t**4

        # Acceleration
        xd_ddot = 2*ax[2] + 6*ax[3]*t + 12*ax[4]*t**2 + 20*ax[5]*t**3
        yd_ddot = 2*ay[2] + 6*ay[3]*t + 12*ay[4]*t**2 + 20*ay[5]*t**3
        zd_ddot = 2*az[2] + 6*az[3]*t + 12*az[4]*t**2 + 20*az[5]*t**3
    else:
        xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot = (
            0, 0, 0, 0, 0, 0, 0, 0, 0)
        print(f'Landed - T:{round(self.t,3)}')
        rospy.signal_shutdown("Trajectory Tracking Complete")
    return xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot
```

The conditional statements provide the trajectories between two points based on current time. It returns trajectory start and end point as follows:
 - $ t < 5 s \rightarrow p_{0} - p_{1}$ 
 - $ 5 s < t < 20 s \rightarrow p_{1} - p_{2}$ 
 - $ 20 s < t < 35 s \rightarrow p_{2} - p_{3}$ 
 - $ 35 s < t < 50 s \rightarrow p_{3} - p_{4}$ 
 - $ 50 s < t < 65 s \rightarrow p_{4} - p_{5}$ 
 - $ 65 s < t < 70 s \rightarrow p_{5} - p_{6}$ 

$p_{6}$ ensures that drone lands so it does not crash when the simulation ends.

Each condition is used to set the values of initial and final variables for trajectory evaulation which include time, position, velocity and acceleration.

```
if self.t <= 5:
    t0, tf, x0, y0, z0, xf, yf, zf, v0, vf, ac0, acf, t = (
        0, 5, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, self.t)
    print(f'Lift OFF - T:{round(self.t,3)}')
elif self.t <= 20:
    t0, tf, x0, y0, z0, xf, yf, zf, v0, vf, ac0, acf, t = (
        5, 20, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, self.t)
    print(f'Path 1 - T:{round(self.t,3)}')
elif self.t <= 35:
    t0, tf, x0, y0, z0, xf, yf, zf, v0, vf, ac0, acf, t = (
        20, 35, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, self.t)
    print(f'Path 2 - T:{round(self.t,3)}')
elif self.t <= 50:
    t0, tf, x0, y0, z0, xf, yf, zf, v0, vf, ac0, acf, t = (
        35, 50, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, self.t)
    print(f'Path 3 - T:{round(self.t,3)}')
elif self.t <= 65:
    t0, tf, x0, y0, z0, xf, yf, zf, v0, vf, ac0, acf, t = (
        50, 65, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, self.t)
    print(f'Path 4 - T:{round(self.t,3)}')
else:
    t0, tf, x0, y0, z0, xf, yf, zf, v0, vf, ac0, acf, t = (
        65, 70, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, self.t)
    print(f'Landing - T:{round(self.t,3)}')
```

Using the initial and final times, the time matrix is generated. This is generated by substituting the initial and final time values in quintic polynomial equation for position and its derivative and double derivative for velocity and acceleration respectively.

```
A = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5],
              [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
              [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
              [1, tf, tf**2, tf ** 3, tf ** 4, tf ** 5],
              [0, 1, 2*tf, 3 * tf ** 2, 4 * tf ** 3, 5 * tf ** 4],
              [0, 0, 2, 6 * tf, 12 * tf ** 2, 20 * tf ** 3]])
```

The `b` matrix is the RHS matrix which holds the value of initial and final position, velocity and acceleration as explained above in the math.

```
bx = np.array(([x0], [v0], [ac0], [xf], [vf], [acf]))
by = np.array(([y0], [v0], [ac0], [yf], [vf], [acf]))
bz = np.array(([z0], [v0], [ac0], [zf], [vf], [acf]))
```

Finally, the coefficients of quintic polynomial are calculated by multiplying the inverse of `time matrix` with `b` matrix

```
ax = np.dot(np.linalg.inv(A), bx)
ay = np.dot(np.linalg.inv(A), by)
az = np.dot(np.linalg.inv(A), bz)
```

Finally the quintic polynomial for is reconstructed using the coefficient and current time. This generates $x_{d}, y_{d}$ and $z_{d}$

```
# Position
xd = ax[0] + ax[1]*t + ax[2]*t**2 + \
    ax[3]*t**3 + ax[4]*t**4 + ax[5]*t**5
yd = ay[0] + ay[1]*t + ay[2]*t**2 + \
    ay[3]*t**3 + ay[4]*t**4 + ay[5]*t**5
zd = az[0] + az[1]*t + az[2]*t**2 + \
    az[3]*t**3 + az[4]*t**4 + az[5]*t**5
```

Similarly $\dot{x_{d}},\dot{y_{d}}$ and $\dot{z_{d}}$ are generated.

```
# Velocity
xd_dot = ax[1] + 2*ax[2]*t + 3*ax[3] * \
    t**2 + 4*ax[4]*t**3 + 5*ax[5]*t**4
yd_dot = ay[1] + 2*ay[2]*t + 3*ay[3] * \
    t**2 + 4*ay[4]*t**3 + 5*ay[5]*t**4
zd_dot = az[1] + 2*az[2]*t + 3*az[3] * \
    t**2 + 4*az[4]*t**3 + 5*az[5]*t**4
```

Similarly $\ddot{x_{d}},\ddot{y_{d}}$ and $\ddot{z_{d}}$ are generated.

```
# Acceleration
xd_ddot = 2*ax[2] + 6*ax[3]*t + 12*ax[4]*t**2 + 20*ax[5]*t**3
yd_ddot = 2*ay[2] + 6*ay[3]*t + 12*ay[4]*t**2 + 20*ay[5]*t**3
zd_ddot = 2*az[2] + 6*az[3]*t + 12*az[4]*t**2 + 20*az[5]*t**3

```
In the condition when the complete trajectory is traced, the origin is returned with zero velocity and acceleration to power off the drone.

```
else:
    xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot = (
        0, 0, 0, 0, 0, 0, 0, 0, 0)
        print(f'Landed - T:{round(self.t,3)}')
```

After trajectory tracking is complete, the ros node is shutdown as there is no further path left to trace.

```
rospy.signal_shutdown("Trajectory Tracking Complete")
```

The complete function returns position, velocity and acceleration

```
return xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot
```

## Part 2: Controller Design

Considering the equations of motion provided above, design boundary layer-based sliding mode control laws for the $z,\phi,\theta,\psi$ coordinates of the quadrotor to track desired trajectories $z_{d}, \phi_{d}, \theta_{d},$ and $\psi_{d}$.

**Solution:** Four sliding mode controller will be needed to control the drone. $s_{1}, s_{2}, s_{3}, s_{4}$ will control $u_{1}, u_{2}, u_{3}, u_{4}$ respectively, which in turn will control  $z, \phi, \theta$, and $\psi$ respectively.

**Designing Controller 1 to control $z$:**

$$
\begin{equation}
s_{1} = \dot{e} + \lambda_{z}e\notag
\end{equation}
$$

where, $\dot{e} = \dot{z} - \dot{z_{d}}$ and ${e} = {z} - z_{d}$

$$
\begin{equation}
\dot{s_{1}} = \ddot{e} + \lambda_{z}\dot{e} \tag{1}
\end{equation}
$$

where, $\ddot{e} = \ddot{z} - \ddot{z_{d}}$ and $\dot{e} = \dot{z} - \dot{z_{d}}$

Also,

$$
\begin{equation}
\ddot{z} = \frac{1}{m}(cos\phi cos\theta)u_{1}-g \tag{2}
\end{equation}
$$

Placing $(2)$ in $(1)$ and calculating $s_{1}\dot{s_{1}}$:

$$
\begin{equation}
s_{1}\dot{s_{1}} = s_{1}(\frac{cos\phi cos\theta}{m}u_{1}-g - \ddot{z_{d}} + \lambda_{z}(\dot{z}-\dot{z_{d}}))\notag
\end{equation}
$$

Making the coefficient of $u_{1}$ as 1 will result in:

$$
\begin{equation}
s_{1}\dot{s_{1}} = s_{1}\frac{cos\phi cos\theta}{m}(u_{1} + \frac{m}{cos\phi cos\theta}(-g - \ddot{z_{d}} + \lambda_{z}(\dot{z}-\dot{z_{d}})))\tag{3}
\end{equation}
$$

The control strategy we came up to is to design a controller which cancels the system dynamics:

$$
\begin{equation}
u_{1} = - \frac{m}{cos\phi cos\theta}(-g - \ddot{z_{d}} + \lambda_{z}(\dot{z}-\dot{z_{d}}) + u_{r}) \tag{4}
\end{equation}
$$

where, $u_{r}$ is the robust term.

Placing equation $(4)$ in equation $(3)$ will result in:

$$
\begin{equation}
s_{1}\dot{s_{1}} = -s_{1}u_{r}\notag
\end{equation}
$$

We are designing the controller with reduced chattering, so we shall be adding a boundary layer of $\gamma$ which is the acceptable tolerance. Thus, assume $u_{r} = K_z sat(s_1)$. Therefore;

$$
\begin{equation}
s_{1}\dot{s_{1}} = -s_{1} K_{z} sat(s_1)\notag
\end{equation}
$$

for $K_z > 0$, the resulting system is always negative, thus asymptotically stable. Thus,

$$
\begin{equation}
u_{1} = - \frac{m}{cos\phi cos\theta}(-g - \ddot{z_{d}} + \lambda_{z}(\dot{z}-\dot{z_{d}}) + K_{z}sat(s_1))\tag{A}
\end{equation}
$$



**Designing Controller 2 to control $\phi$:**

$$
\begin{equation}
s_{2} = \dot{e} + \lambda_{\phi}e\notag
\end{equation}
$$

where, $\dot{e} = \dot{\phi} - \dot{\phi_{d}}$ and ${e} = {\phi} - \phi_{d}$

$$
\begin{equation}
\dot{s_{2}} = \ddot{e} + \lambda_{\phi}\dot{e} \tag{5}
\end{equation}
$$

where, $\ddot{e} = \ddot{\phi} - \ddot{\phi_{d}}$ and $\dot{e} = \dot{\phi} - \dot{\phi_{d}}$

Also,

$$
\begin{equation}
\ddot{\phi} =\dot{\theta}\dot{\psi}\frac{I_{y}-I_{z}}{I_{x}}-\frac{I_{p}}{I_{x}}\Omega \dot{\theta}+\frac{1}{I_{x}}u_{2}\tag{6}
\end{equation}
$$

Placing $(6)$ in $(5)$ and calculating $s_{2}\dot{s_{2}}$:

$$
\begin{equation}
s_{2}\dot{s_{2}} = s_{2}(\dot{\theta}\dot{\psi}\frac{I_{y}-I_{z}}{I_{x}}-\frac{I_{p}}{I_{x}}\Omega \dot{\theta}+\frac{1}{I_{x}}u_{2}-\ddot{\phi_{d}} + \lambda_{\phi}(\dot{\phi} - \dot{\phi_{d}}))\notag
\end{equation}
$$

Making the coefficient of $u_{2}$ as 1 will result in:

$$
\begin{equation}
s_{2}\dot{s_{2}} = \frac{s_{2}}{I_{x}}(u_{2} + (\dot{\theta}\dot{\psi}(I_{y}-I_{z})-I_{p}\Omega \dot{\theta}-I_{x}\ddot{\phi_{d}} + I_{x}\lambda_{\phi}(\dot{\phi} - \dot{\phi_{d}})))\tag{7}
\end{equation}
$$

The control strategy we came up to is to design a controller which cancels the system dynamics:

$$
\begin{equation}
u_{2} = -(\dot{\theta}\dot{\psi}(I_{y}-I_{z})-I_{p}\Omega \dot{\theta}-I_{x}\ddot{\phi_{d}} + I_{x}\lambda_{\phi}(\dot{\phi} - \dot{\phi_{d}}) + I_{x}u_{r})\tag{8}
\end{equation}
$$

where, $u_{r}$ is the robust term.

Placing equation $(8)$ in equation $(7)$ will result in:

$$
\begin{equation}
s_{2}\dot{s_{2}} = -s_{2}u_{r}\notag
\end{equation}
$$

We are designing the controller with reduced chattering, so we shall be adding a boundary layer of $\gamma$ which is the acceptable tolerance. Thus, assume $u_{r} = K_{\phi} sat(s_2)$. Therefore;

$$
\begin{equation}
s_{2}\dot{s_{2}} = -s_{2} K_{\phi} sat(s_2)\notag
\end{equation}
$$

for $K_{\phi} > 0$, the resulting system is always negative, thus asymptotically stable. Thus,

$$
\begin{equation}
u_{2} = -(\dot{\theta}\dot{\psi}(I_{y}-I_{z})-I_{p}\Omega \dot{\theta}-I_{x}\ddot{\phi_{d}} + I_{x}\lambda_{\phi}(\dot{\phi} - \dot{\phi_{d}}) + I_{x}K_{\phi} sat(s_2))\tag{B}
\end{equation}
$$

**Designing Controller 3 to control $\theta$:**

$$
\begin{equation}
s_{3} = \dot{e} + \lambda_{\theta}e\notag
\end{equation}
$$

where, $\dot{e} = \dot{\theta} - \dot{\theta_{d}}$ and ${e} = {\theta} - \theta_{d}$

$$
\begin{equation}
\dot{s_{3}} = \ddot{e} + \lambda_{\theta}\dot{e} \tag{9}
\end{equation}
$$

where, $\ddot{e} = \ddot{\theta} - \ddot{\theta_{d}}$ and $\dot{e} = \dot{\theta} - \dot{\theta_{d}}$

Also,

$$
\begin{equation}
\ddot{\theta} =\dot{\phi}\dot{\psi}\frac{I_{z}-I_{x}}{I_{y}}+\frac{I_{p}}{I_{y}}\Omega \dot{\phi}+\frac{1}{I_{y}}u_{3}\tag{10}
\end{equation}
$$

Placing $(10)$ in $(9)$ and calculating $s_{3}\dot{s_{3}}$:

$$
\begin{equation}
s_{3}\dot{s_{3}} = s_{3}(\dot{\phi}\dot{\psi}\frac{I_{z}-I_{x}}{I_{y}}+\frac{I_{p}}{I_{y}}\Omega \dot{\phi}+\frac{1}{I_{y}}u_{3}-\ddot{\theta_{d}} + \lambda_{\theta}(\dot{\theta} - \dot{\theta_{d}}))\notag
\end{equation}
$$

Making the coefficient of $u_{3}$ as 1 will result in:

$$
\begin{equation}
s_{3}\dot{s_{3}} = \frac{s_{3}}{I_{y}}(u_{3} + (\dot{\phi}\dot{\psi}(I_{z}-I_{x})+I_{p}\Omega \dot{\phi}-I_{y}\ddot{\theta_{d}} + I_{y}\lambda_{\theta}(\dot{\theta} - \dot{\theta_{d}})))\tag{11}
\end{equation}
$$

The control strategy we came up to is to design a controller which cancels the system dynamics:

$$
\begin{equation}
u_{3} = -(\dot{\phi}\dot{\psi}(I_{z}-I_{x})+I_{p}\Omega \dot{\phi}-I_{y}\ddot{\theta_{d}} + I_{y}\lambda_{\theta}(\dot{\theta} - \dot{\theta_{d}}) + I_{y}u_{r})\tag{12}
\end{equation}
$$

where, $u_{r}$ is the robust term.

Placing equation $(12)$ in equation $(11)$ will result in:

$$
\begin{equation}
s_{3}\dot{s_{3}} = -s_{3}u_{r}\notag
\end{equation}
$$

We are designing the controller with reduced chattering, so we shall be adding a boundary layer of $\gamma$ which is the acceptable tolerance. Thus, assume $u_{r} = K_{\theta} sat(s_3)$. Therefore;

$$
\begin{equation}
s_{3}\dot{s_{3}} = -s_{3} K_{\theta} sat(s_3)\notag
\end{equation}
$$

for $K_{\theta} > 0$, the resulting system is always negative, thus asymptotically stable. Thus,

$$
\begin{equation}
u_{3} = -(\dot{\phi}\dot{\psi}(I_{z}-I_{x})+I_{p}\Omega \dot{\phi}-I_{y}\ddot{\theta_{d}} + I_{y}\lambda_{\theta}(\dot{\theta} - \dot{\theta_{d}}) + I_{y}K_{\theta} sat(s_3))\tag{C}
\end{equation}
$$


# What each scripts contain?

# Packages used
- [Symbolic Python (sympy)](https://github.com/sympy/sympy)
- matplotlib

# Designer Details

- Designed for:
  - Worcester Polytechnic Institute
  - RBE 502: Robot Control - Final Project
- Designed by:
  - [Parth Patel](mailto:parth.pmech@gmail.com)
  - [Prarthana Sigedar](mailto:prarthana.sigedar@gmail.com)

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
