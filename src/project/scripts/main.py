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
        self.odom_sub = rospy.Subscriber(
            "/crazyflie2/ground_truth/odometry", Odometry, self.odom_callback)

        self.t0 = None
        self.t = None
        self.t_series = []
        self.x_series = []
        self.y_series = []
        self.z_series = []
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

        self.w_max = 2168  # maximum rotary speed
        self.w_min = 0  # minimum rotation speed

        self.u1 = 0
        self.u2 = 0
        self.u3 = 0
        self.u4 = 0

        self.w1 = 0
        self.w2 = 0
        self.w3 = 0
        self.w4 = 0

        #!: Tune the Kp and Kd Values here
        self.kpx = 0
        self.kdx = 0
        self.kpy = 0
        self.kdy = 0

        #!: Tune the lambda values
        self.lamda1 = 0
        self.lamda2 = 0
        self.lamda3 = 0
        self.lamda4 = 0

    def traj_evaluate(self):
        # TODO: Trajectory Inputs
        if (self.t < 5):
            pos1 = np.array([0, 0, 0.0800*self.t**3 - 0.0240 *
                            self.t**4 + 0.0019*self.t**5])
            vel1 = np.array([0, 0, 0.2400*self.t**2 - 0.0960 *
                            self.t**3 + 0.0096*self.t**4])
            acc1 = np.array([0, 0, 0.4800*self.t - 0.2880 *
                            self.t**2 + 0.0384*self.t**3])
            return pos1, vel1, acc1
        elif (self.t < 20):
            pos2 = np.array([0.0030*self.t**3 - 0.00029630*self.t **
                            4 + 0.0000079012*self.t**5, 0, 1])
            vel2 = np.array([0, 0, 0.0800 * self.t ** 3 - 0.0240 *
                            self.t ** 4 + 0.0019 * self.t ** 5])
            acc2 = np.array([0.0178*self.t - 0.0036*self.t **
                            2 + 0.00015802*self.t**3, 0, 0])
            return pos2, vel2, acc2
        elif (self.t < 35):
            pos3 = np.array([1, 0.0030 * self.t ** 3 - 0.00029630 *
                            self.t ** 4 + 0.0000079012 * self.t ** 5, 1])
            vel3 = np.array([0, 0.0089*self.t**2 - 0.0012 *
                            self.t**3 + 0.000039506*self.t**4, 0])
            acc3 = np.array([0, 0.0178 * self.t - 0.0036 * self.t **
                            2 + 0.00015802 * self.t ** 3, 0])
            return pos3, vel3, acc3
        elif (self.t < 50):
            pos4 = np.array([0, 0.0178*self.t - 0.0036*self.t **
                            2 + 0.00015802*self.t**3, 0])
            vel4 = np.array([- 0.0089*self.t**2 + 0.0012*self.t **
                            3 - 0.000039506*self.t**4, 0, 0])
            acc4 = np.array([- 0.0178*self.t + 0.0036*self.t **
                            2 - 0.00015802*self.t**3, 0, 0])
            return pos4, vel4, acc4
        elif (self.t < 65):
            pos5 = np.array([0, 1 - 0.0030*self.t**3 + 0.00029630 *
                            self.t**4 - 0.0000079012*self.t**5, 1])
            vel5 = np.array([0, - 0.0089*self.t**2 + 0.0012 *
                            self.t**3 - 0.000039506*self.t**4, 0])
            acc5 = np.array([0, - 0.0178 * self.t + 0.0036 * self.t **
                            2 - 0.00015802 * self.t ** 3, 0])
            return pos5, vel5, acc5
        else:
            return exit

    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
        # obtain the desired values by evaluating the corresponding trajectories
        posd, veld, accd = self.traj_evaluate()
        posd = np.round(posd, 2)
        veld = np.round(veld, 2)
        accd = np.round(accd, 2)

        # !: implement the Sliding Mode Control laws designed in Part 2 to calculate the control inputs "u"
        # TODO: Writing the F_x and F_y to calculate theta_d and phi_d
        force_x = self.m * \
            ((-self.kpx*(xyz[0] - posd[0])) +
             (-self.kdx*(xyz_dot[0] - veld[0])) + accd[0])
        force_y = self.m * \
            ((-self.kpy*(xyz[1] - posd[1])) +
             (-self.kdy*(xyz_dot[1] - veld[1])) + accd[1])
        theta_d = np.arcsin(force_x/self.u1)
        phi_d = np.arcsin(-force_y/self.u1)

        # REMARK: wrap the roll-pitch-yaw angle errors to [-pi to pi]
        # TODO: convert the desired control inputs "u" to desired rotor velocities "motor_vel" by using the "allocation matrix"
        allocation_matrix = np.matrix(
            [
                1/(4*self.kf),
                ((-sqrt(2))/(4*self.kf*self.l)),
                ((-sqrt(2))/(4*self.kf*self.l)),
                (1/(4*self.kf*self.km))
            ],
            [
                1/(4*self.kf),
                ((-sqrt(2))/(4*self.kf*self.l)),
                ((sqrt(2))/(4*self.kf*self.l)),
                (1/(4*self.kf*self.km))
            ],
            [
                1/(4*self.kf),
                ((sqrt(2))/(4*self.kf*self.l)),
                ((sqrt(2))/(4*self.kf*self.l)),
                (-1/(4*self.kf*self.km))
            ],
            [
                1/(4*self.kf),
                ((sqrt(2))/(4*self.kf*self.l)),
                ((-sqrt(2))/(4*self.kf*self.l)),
                (1/(4*self.kf*self.km))
            ],
        )
        u_matrix = np.matrix([self.u1], [self.u2], [self.u3], [self.u4])
        angular_velocity_matrix = sqrt(np.matmul(allocation_matrix, u_matrix))
        self.w1 = angular_velocity_matrix[0]
        self.w2 = angular_velocity_matrix[1]
        self.w3 = angular_velocity_matrix[2]
        self.w4 = angular_velocity_matrix[3]

        # !: maintain the rotor velocities within the valid range of [0 to 2618]
        # publish the motor velocities to the associated ROS topic

        motor_speed = Actuators()
        # note we need to convert in motor velocities
        # motor_speed.angular_velocities = [motor_vel[0, 0], motor_vel[1, 0], motor_vel[2, 0], motor_vel[3, 0]]
        # motor_speed.angular_velocities = [1, 1,0.5,0.1]

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
        # call the controller with the current states
        self.smc_control(xyz, xyz_dot, rpy, rpy_dot)  # sendinf

    # save the actual trajectory data
    def save_data(self):
        # !: update the path below with the correct path. Bring Relative Path
        with open("/home/prar/rbe502_project/src/project/scripts/log.pkl", "wb") as fp:
            self.mutex_lock_on = True
            pickle.dump([self.t_series, self.x_series,
                        self.y_series, self.z_series], fp)


if __name__ == '__main__':
    rospy.init_node("quadrotor_control")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Quadrotor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
