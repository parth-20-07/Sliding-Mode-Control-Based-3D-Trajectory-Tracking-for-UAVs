#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin
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
        # TODO: include initialization codes if needed

        # defining the physical paramters
        self.m = 27/1000
        self.l = 46/1000
        self.g = 9.81

        self.Ix = 16.571710 * (10**(-6))
        self.Iy = 16.571710 * (10**(-6))
        self.Iz = 29.261652 * (10**(-6))
        self.Ip = 12.65625 * (10**(-8))

        self.kf = 1.28192*(10**(-8))
        self.km = 5.964552*(10**(-3))

        self.w_max = 2618  # maximum rotary speed
        self.w_min = 0  # minimum rotation speed

        self.ohm = 0
        #!: Tune the Kp and Kd Values here
        self.kpx = 2
        self.kdx = 3
        self.kpy = 2
        self.kdy = 3

        #!: Tune the lambda values
        self.lamda1 = 10
        self.lamda2 = 0
        self.lamda3 = 0
        self.lamda4 = 0

        self.k1 = 1  # 20
        self.k2 = 0  # 35
        self.k3 = 0  # 50
        self.k4 = 0  # 45

        self.theta_d = 0
        self.phi_d = 0
        self.posd = np.array([0, 0, 0])
        self.motor_vel = np.array([0, 0, 0, 0])

        self.calculate_k = True
        self.round_pi = round(pi, 5)

        # Acceptable Tolerance
        self.tol = 0.0001

    def traj_evaluate(self):
        # TODO: Trajectory Inputs
        if (self.t < 5):
            t = self.t
            print("traj1")
            pos1 = np.array([0, 0, 0.0800*t**3 - 0.0240 *
                             t**4 + 0.0019*t**5])
            vel1 = np.array([0, 0, 0.2400*t**2 - 0.0960 *
                             t**3 + 0.0096*t**4])
            acc1 = np.array([0, 0, 0.4800*t - 0.2880 *
                             t**2 + 0.0384*t**3])
            return pos1, vel1, acc1
        # elif (self.t < 20):
        #     t = self.t - 5
        #     print("traj2")
        #     pos2 = np.array([0.0030*t**3 - 0.00029630*t **
        #                      4 + 0.0000079012*t**5, 0, 1])
        #     vel2 = np.array([0, 0, 0.0800 * t ** 3 - 0.0240 *
        #                      t ** 4 + 0.0019 * t ** 5])
        #     acc2 = np.array([0.0178*t - 0.0036*t **
        #                      2 + 0.00015802*t**3, 0, 0])
        #     return pos2, vel2, acc2
        # elif (self.t < 35):
        #     t = self.t - 20
        #     print("traj3")
        #     pos3 = np.array([1, 0.0030 * t ** 3 - 0.00029630 *
        #                      t ** 4 + 0.0000079012 * t ** 5, 1])
        #     vel3 = np.array([0, 0.0089*t**2 - 0.0012 *
        #                      t**3 + 0.000039506*t**4, 0])
        #     acc3 = np.array([0, 0.0178 * t - 0.0036 * t **
        #                      2 + 0.00015802 * t ** 3, 0])
        #     return pos3, vel3, acc3
        # elif (self.t < 50):
        #     t = self.t - 35
        #     print("traj4")
        #     pos4 = np.array([0, 0.0178*t - 0.0036*t **
        #                      2 + 0.00015802*t**3, 0])
        #     vel4 = np.array([- 0.0089*t**2 + 0.0012*t **
        #                      3 - 0.000039506*t**4, 0, 0])
        #     acc4 = np.array([- 0.0178*t + 0.0036*t **
        #                      2 - 0.00015802*t**3, 0, 0])
        #     return pos4, vel4, acc4
        # elif (self.t < 65):
        #     t = self.t - 50
        #     print("traj5")
        #     pos5 = np.array([0, 1 - 0.0030*t**3 + 0.00029630 *
        #                      t**4 - 0.0000079012*t**5, 1])
        #     vel5 = np.array([0, - 0.0089*t**2 + 0.0012 *
        #                      t**3 - 0.000039506*t**4, 0])
        #     acc5 = np.array([0, - 0.0178 * t + 0.0036 * t **
        #                      2 - 0.00015802 * t ** 3, 0])
        #     return pos5, vel5, acc5
        else:
            return [0, 0, 1], [0, 0, 0], [0, 0, 0]

    def saturation(self, sliding_function):
        sat = 1
        if (sliding_function >= self.tol):
            sat = 1
        elif (sliding_function <= -self.tol):
            sat = -1
        else:
            sat = sliding_function/self.tol
        # print(f"sat: {sat}")
        return sat

    def wrap_to_pi(self, angle):
        angle = np.round(angle, 5)
        if ((angle >= self.round_pi) or (angle <= -self.round_pi)):
            angle = angle % self.round_pi
        return angle

    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
        pos_x = xyz[0]
        pos_y = xyz[1]
        pos_z = xyz[2]
        vel_x = xyz_dot[0]
        vel_y = xyz_dot[1]
        vel_z = xyz_dot[2]
        phi = self.wrap_to_pi(rpy[0])
        theta = self.wrap_to_pi(rpy[1])
        psi = self.wrap_to_pi(rpy[2])
        dphi = rpy_dot[0]
        dtheta = rpy_dot[1]
        dpsi = rpy_dot[2]

        # obtain the desired values by evaluating the corresponding trajectories
        self.posd, veld, accd = self.traj_evaluate()
        self.posd = np.round(self.posd, 5)
        veld = np.round(veld, 5)
        accd = np.round(accd, 5)
        pos_x_des = self.posd[0]
        pos_y_des = self.posd[1]
        pos_z_des = self.posd[2]
        vel_x_des = veld[0]
        vel_y_des = veld[1]
        vel_z_des = veld[2]
        acc_x_des = accd[0]
        acc_y_des = accd[1]
        acc_z_des = accd[2]

        # TODO: implement the Sliding Mode Control laws designed in Part 2 to calculate the control inputs "u"
        s1_error_dot = np.round(vel_z-vel_z_des, 5)
        s1_error = np.round(pos_z-pos_z_des, 5)
        s1 = (s1_error_dot) + self.lamda1*(s1_error)

        u1 = (self.m/(cos(theta)*cos(phi))) * \
            (self.g + acc_z_des - (self.lamda1*s1_error_dot) -
             (self.k1*self.saturation(s1)))

        # TODO: Writing the F_x and F_y to calculate theta_d and phi_d
        x_error = np.round(pos_x - pos_x_des, 5)
        x_error_dot = np.round(vel_x - vel_x_des, 5)
        y_error = np.round(pos_y - pos_y_des, 5)
        y_error_dot = np.round(vel_y - vel_y_des, 5)
        force_x = self.m * ((-self.kpx*x_error) +
                            (-self.kdx*x_error_dot) + acc_x_des)
        force_y = self.m * ((-self.kpy*y_error) +
                            (-self.kdy*y_error_dot) + acc_y_des)
        sin_theta_des = force_x/u1
        sin_phi_des = -force_y/u1
        print(f"Des: theta_des:{sin_theta_des} | phi_des:{sin_phi_des}")
        theta_des = self.wrap_to_pi(np.arcsin(sin_theta_des))
        phi_des = self.wrap_to_pi(np.arcsin(sin_phi_des))

        s2_error_dot = np.round(dphi, 5)
        s2_error = np.round(phi - phi_des, 5)
        s2 = (s2_error_dot) + self.lamda2*(s2_error)
        u2 = - ((dtheta*dpsi*(self.Iy-self.Iz))-(self.Ip*self.ohm*dtheta) +
                (self.lamda2*self.Ix*s2_error_dot)+(self.Ix*self.k2*self.saturation(s2)))

        s3_error_dot = np.round(dtheta, 5)
        s3_error = np.round(theta-theta_des, 5)
        s3 = (s3_error_dot) + self.lamda1*(s3_error)
        u3 = -((dphi*dpsi*(self.Iy-self.Ix))+(self.Ip*self.ohm*dphi) +
               (self.Iy*self.lamda3*s3_error_dot)+(self.Iy*self.k3*self.saturation(s3)))

        s4_error_dot = np.round(dpsi, 5)
        s4_error = np.round(psi, 5)
        s4 = (s4_error_dot) + self.lamda1*(s4_error)
        u4 = -((dphi*dtheta*(self.Ix-self.Iy))+(self.lamda4 *
                                                self.Iz*s4_error_dot)+(self.Iz*self.k4*self.saturation(s4)))

        u1 = np.round(u1, 4)
        u2 = np.round(u2, 4)
        u3 = np.round(u3, 4)
        u4 = np.round(u4, 4)

        print(
            f"Error: t: {self.t} | z:{s1_error[0]} | y:{y_error[0]} | phi:{s2_error[0]} | x: {x_error[0]} | theta: {s3_error[0]} | psi:{s4_error[0]}")
        print(f"U: {u1} | {u2} | {u3} | {u4}")

        # TODO: convert the desired control inputs "u" to desired rotor velocities "self.motor_vel" by using the "allocation matrix"
        aa = 1/(4*self.kf)
        bb = sqrt(2)/(4*self.kf*self.l)
        cc = 1/(4*self.km*self.kf)
        w1 = round(sqrt((aa*u1)-(bb*u2)-(bb*u3)-(cc*u4)))
        w2 = round(sqrt((aa*u1)-(bb*u2)+(bb*u3)+(cc*u4)))
        w3 = round(sqrt((aa*u1)+(bb*u2)+(bb*u3)-(cc*u4)))
        w4 = round(sqrt((aa*u1)+(bb*u2)-(bb*u3)+(cc*u4)))
        self.motor_vel = [w1, w2, w3, w4]
        self.ohm = w1-w2+w3-w4

        # TODO: maintain the rotor velocities within the valid range of [0 to 2618]
        for i in range(4):
            if (self.motor_vel[i] > self.w_max):
                self.motor_vel[i] = self.w_max
        print(
            f"w1: {self.motor_vel[0]} | w2: {self.motor_vel[1]} | w3: {self.motor_vel[2]} | w4: {self.motor_vel[3]}")
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
        # TODO: update the path below with the correct path. Bring Relative Path
        # with open("log.pkl", "wb") as fp:
        #     self.mutex_lock_on = True
        #     pickle.dump([self.t_series, self.x_series,
        #                  self.y_series, self.z_series, self.traj_x_series, self.traj_y_series, self.traj_z_series], fp)
        # with open("omega_log.pkl", "wb") as fp:
        #     self.mutex_lock_on = True
        #     pickle.dump([self.t_series, self.w1_series,
        #                 self.w2_series, self.w3_series, self.w4_series], fp)
        # print("Visualizing System Plot")
        # os.system("python3 visualize.py")
        # os.system("python3 visualize_omega.py")


if __name__ == '__main__':
    if os.path.exists("log.pkl"):
        os.remove("log.pkl")
        os.remove("trajectory.png")
    if os.path.exists("omega_log.pkl"):
        os.remove("omega_log.pkl")
        os.remove("omega.png")
    rospy.init_node("quadrotor_control")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Quadrotor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
