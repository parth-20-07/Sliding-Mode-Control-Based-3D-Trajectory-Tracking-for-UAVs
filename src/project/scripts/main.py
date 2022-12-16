#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin, asin
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import sympy as sym
import os


class Quadrotor():
    def __init__(self):
        # publisher for rotor speeds
        self.motor_speed_pub = rospy.Publisher(
            "/crazyflie2/command/motor_speed", Actuators, queue_size=10)

        # subscribe to Odometry topic
        self.odom_sub = self.odom_sub = rospy.Subscriber(
            "/crazyflie2/ground_truth/odometry", Odometry, self.odom_callback, queue_size=1)

        self.t0 = None
        self.t = None
        self.t_series = []
        self.x_series = []
        self.y_series = []
        self.z_series = []
        self.traj_x_series = []
        self.traj_y_series = []
        self.traj_z_series = []
        self.w1_series = []
        self.w2_series = []
        self.w3_series = []
        self.w4_series = []
        self.mutex_lock_on = False
        rospy.on_shutdown(self.save_data)

        # defining the physical paramters
        self.m = 27/1000
        self.l = 46/1000
        self.g = 9.81

        self.Ix = 16.571710 * (10**(-6))
        self.Iy = 16.571710 * (10**(-6))
        self.Iz = 29.261652 * (10**(-6))
        self.Ip = 12.65625 * (10**(-8))

        self.k_f = 1.28192*(10**(-8))
        self.k_m = 5.964552*(10**(-3))

        self.w_max = 2618  # maximum rotary speed
        self.w_min = 0  # minimum rotation speed

        self.ohm = 0
        #!: Tune the Kp and Kd Values here
        self.kp = 50
        self.kd = 5

        #!: Tune the lambda values
        self.lamda_z = 5
        self.lamda_phi = 10
        self.lamda_theta = 10
        self.lamda_psi = 5

        self.k_z = 25
        self.k_phi = 150
        self.k_theta = 150
        self.k_psi = 20

        self.posd = np.array([0, 0, 0])
        self.motor_vel = np.array([0, 0, 0, 0])

        # Acceptable Tolerance
        self.tol = 0.01

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

    def saturation(self, sliding_function):
        sat = min(max(sliding_function/self.tol, -1), 1)
        return sat

    def wrap_to_pi(self, angle):
        return (angle+np.pi) % (2 * np.pi)-np.pi

    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
        pos_x, pos_y, pos_z = xyz
        vel_x, vel_y, vel_z = xyz_dot
        phi, theta, psi = rpy
        dphi, dtheta, dpsi = rpy_dot

        # obtain the desired values by evaluating the corresponding trajectories
        xd, yd, zd, xd_dot, yd_dot, zd_dot, xd_ddot, yd_ddot, zd_ddot = self.traj_evaluate()
        self.posd = [xd, yd, zd]

        s1_error_dot = vel_z-zd_dot
        s1_error = pos_z-zd
        s1 = (s1_error_dot) + self.lamda_z*(s1_error)

        u1 = self.m * (self.g + zd_ddot - (self.lamda_z*s1_error_dot) -
                       (self.k_z*self.saturation(s1)))/(cos(theta)*cos(phi))

        x_error = pos_x - xd
        x_error_dot = vel_x - xd_dot
        y_error = pos_y - yd
        y_error_dot = vel_y - yd_dot
        force_x = self.m * ((-self.kp*x_error) +
                            (-self.kd*x_error_dot) + xd_ddot)
        force_y = self.m * ((-self.kp*y_error) +
                            (-self.kd*y_error_dot) + yd_ddot)
        sin_theta_des = force_x/u1
        sin_phi_des = -force_y/u1
        # print(f"Des: theta_des:{sin_theta_des} | phi_des:{sin_phi_des}")
        theta_des = asin(sin_theta_des)
        phi_des = asin(sin_phi_des)

        s2_error_dot = dphi
        s2_error = self.wrap_to_pi(phi - phi_des)
        s2 = (s2_error_dot) + self.lamda_phi*(s2_error)
        u2 = - ((dtheta*dpsi*(self.Iy-self.Iz))-(self.Ip*self.ohm*dtheta)
                + (self.lamda_phi*self.Ix*s2_error_dot)+(self.Ix*self.k_phi*self.saturation(s2)))

        s3_error_dot = dtheta
        s3_error = self.wrap_to_pi(theta-theta_des)
        s3 = (s3_error_dot) + self.lamda_z*(s3_error)
        u3 = -((dphi*dpsi*(self.Iz-self.Ix))+(self.Ip*self.ohm*dphi)
               + (self.Iy*self.lamda_theta*s3_error_dot)+(self.Iy*self.k_theta*self.saturation(s3)))

        s4_error_dot = dpsi
        s4_error = self.wrap_to_pi(psi)
        s4 = (s4_error_dot) + self.lamda_z*(s4_error)
        u4 = -((dphi*dtheta*(self.Ix-self.Iy)) +
               (self.lamda_psi * self.Iz*s4_error_dot)+(self.Iz*self.k_psi*self.saturation(s4)))

        # print(
        # f"Error: z:{s1_error[0]} | y:{y_error[0]} | phi:{s2_error[0]} | x: {x_error[0]} | theta: {s3_error[0]} | psi:{s4_error[0]}")
        # t: {self.t} |
        # print(f"U: {u1} | {u2} | {u3} | {u4}")

        u_mat = np.array([u1, u2, u3, u4])

        # Allocation matrix
        alloc_mat = np.array(([1/(4*self.k_f), -sqrt(2)/(4*self.k_f*self.l), -sqrt(2)/(4*self.k_f*self.l), -1/(4*self.k_m*self.k_f)],
                              [1/(4*self.k_f), -sqrt(2)/(4*self.k_f*self.l),
                               sqrt(2)/(4*self.k_f*self.l),  1/(4*self.k_m*self.k_f)],
                              [1/(4*self.k_f),  sqrt(2)/(4*self.k_f*self.l),
                               sqrt(2)/(4*self.k_f*self.l), -1/(4*self.k_m*self.k_f)],
                              [1/(4*self.k_f),  sqrt(2)/(4*self.k_f*self.l), -sqrt(2)/(4*self.k_f*self.l),  1/(4*self.k_m*self.k_f)]))

        self.motor_vel = np.sqrt(np.dot(alloc_mat, u_mat))
        for i in range(4):
            if (self.motor_vel[i] > self.w_max):
                self.motor_vel[i] = self.w_max
        self.ohm = self.motor_vel[0]-self.motor_vel[1] + \
            self.motor_vel[2]-self.motor_vel[2]

        # print(
        # f"w1: {self.motor_vel[0]} | w2: {self.motor_vel[1]} | w3: {self.motor_vel[2]} | w4: {self.motor_vel[3]}")
        # publish the motor velocities to the associated ROS topic

        motor_speed = Actuators()
        # note we need to convert in motor velocities
        motor_speed.angular_velocities = [
            self.motor_vel[0], self.motor_vel[1], self.motor_vel[2], self.motor_vel[3]]
        self.motor_speed_pub.publish(motor_speed)

        # odometry callback function (DO NOT MODIFY)
    def odom_callback(self, msg):
        if self.t0 == None:
            self.t0 = msg.header.stamp.to_sec()
        self.t = msg.header.stamp.to_sec() - self.t0
        # convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
        w_b = np.asarray([[msg.twist.twist.angular.x], [
            msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
        v_b = np.asarray([[msg.twist.twist.linear.x], [
            msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
        xyz = np.asarray([[msg.pose.pose.position.x], [
            msg.pose.pose.position.y], [msg.pose.pose.position.z]])
        q = msg.pose.pose.orientation

        T = tf.transformations.quaternion_matrix(
            [q.x, q.y, q.z, q.w])  # converting into axisangle rotation
        # fitting the translation vector in homogenous transformation matrix
        T[0:3, 3] = xyz[0:3, 0]
        R = T[0:3, 0:3]

        xyz_dot = np.dot(R, v_b)
        rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
        rpy_dot = np.dot(np.asarray([
            [1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],
            [0, np.cos(rpy[0]), -np.sin(rpy[0])],
            [0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]
        ]), w_b)
        rpy = np.expand_dims(rpy, axis=1)

        # store the actual trajectory to be visualized later
        if (self.mutex_lock_on is not True):
            self.t_series.append(self.t)
            self.x_series.append(xyz[0, 0])
            self.y_series.append(xyz[1, 0])
            self.z_series.append(xyz[2, 0])
            self.traj_x_series.append(self.posd[0])
            self.traj_y_series.append(self.posd[1])
            self.traj_z_series.append(self.posd[2])
            self.w1_series.append(self.motor_vel[0])
            self.w2_series.append(self.motor_vel[1])
            self.w3_series.append(self.motor_vel[2])
            self.w4_series.append(self.motor_vel[3])
        # call the controller with the current states
        self.smc_control(xyz, xyz_dot, rpy, rpy_dot)  # sendinf

        rospy.Rate(100)

    # save the actual trajectory data
    def save_data(self):
        os.system("")
        with open("src/project/scripts/log.pkl", "wb") as fp:
            self.mutex_lock_on = True
            pickle.dump([self.t_series, self.x_series,
                         self.y_series, self.z_series, self.traj_x_series, self.traj_y_series, self.traj_z_series], fp)
        print("Visualizing System Plot")
        os.system("rosrun project visualize.py")


if __name__ == '__main__':
    if os.path.exists("src/project/scripts/log.pkl"):
        os.remove("src/project/scripts/log.pkl")
        os.remove("src/project/scripts/trajectory.png")
    rospy.init_node("quadrotor_control")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Quadrotor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
