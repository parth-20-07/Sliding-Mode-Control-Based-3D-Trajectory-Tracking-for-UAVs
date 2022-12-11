import os
import sympy as sym
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
import array as ar


def traj_calculator(initial_location, final_location, t0, tf):
    t = sym.Symbol('t')
    # T*A = P format
    time_array = np.matrix([  # Making T
        [1, t0, np.power(t0, 2), np.power(t0, 3),
         np.power(t0, 4), np.power(t0, 5)],
        [0, 1, 2*t0, 3*np.power(t0, 2), 4*np.power(t0, 3), 5*np.power(t0, 4)],
        [0, 0, 2, 6*t0, 12*np.power(t0, 2), 20*np.power(t0, 3)],
        [1, tf, np.power(tf, 2), np.power(tf, 3),
         np.power(tf, 4), np.power(tf, 5)],
        [0, 1, 2*tf, 3*np.power(tf, 2), 4*np.power(tf, 3), 5*np.power(tf, 4)],
        [0, 0, 2, 6*tf, 12*np.power(tf, 2), 20*np.power(tf, 3)],
    ])
    # Making P
    x = np.matrix([
        [initial_location[0]], [0], [0],
        [final_location[0]], [0], [0]])
    y = np.matrix([
        [initial_location[1]], [0], [0],
        [final_location[1]], [0], [0]])
    z = np.matrix([
        [initial_location[2]], [0], [0],
        [final_location[2]], [0], [0]])
    # A = T^-1 * P
    T_inv = inv(time_array)
    coefficient_matrix_x = np.matmul(T_inv, x)
    coefficient_matrix_y = np.matmul(T_inv, y)
    coefficient_matrix_z = np.matmul(T_inv, z)
    t_mat = np.matrix(
        [[1, t, np.power(t, 2), np.power(t, 3), np.power(t, 4), np.power(t, 5)]])
    x_traj = np.matmul(t_mat, coefficient_matrix_x)
    y_traj = np.matmul(t_mat, coefficient_matrix_y)
    z_traj = np.matmul(t_mat, coefficient_matrix_z)
    return [x_traj, y_traj, z_traj]


def drange(start, stop, step):
    while start < stop:
        yield start
        start += step


def plot_trajectories(trajectory, t0, tf):
    traj = [0, 0, 0]
    t = sym.Symbol('t')
    for i in drange(t0, tf, 0.5):
        traj[i*2][0] = trajectory[0][0].subs(t, i)
        traj[i*2][1] = trajectory[1][0].subs(t, i)
        traj[i*2][2] = trajectory[2][0].subs(t, i)
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(traj[:, 0], traj[:, 1], traj[:, 2], 'gray')


def main():
    p0 = np.array([0, 0, 0])
    p1 = np.array([0, 0, 1])
    p2 = np.array([1, 0, 1])
    p3 = np.array([1, 1, 1])
    p4 = np.array([0, 1, 1])
    p5 = np.array([0, 0, 1])
    initial_time = 0
    time_for_p0_to_p1 = 5
    time_for_rest_points = 15
    traj1 = traj_calculator(p0, p1, initial_time, time_for_p0_to_p1)
    print(f'Traj1 -> x: {traj1[0]} | y: {traj1[1]} | z: {traj1[2]}')
    traj2 = traj_calculator(p1, p2, initial_time, time_for_rest_points)
    print(f'Traj2 -> x: {traj2[0]} | y: {traj2[1]} | z: {traj2[2]}')
    traj3 = traj_calculator(p2, p3, initial_time, time_for_rest_points)
    print(f'Traj3 -> x: {traj3[0]} | y: {traj3[1]} | z: {traj3[2]}')
    traj4 = traj_calculator(p3, p4, initial_time, time_for_rest_points)
    print(f'Traj4 -> x: {traj4[0]} | y: {traj4[1]} | z: {traj4[2]}')
    traj5 = traj_calculator(p4, p5, initial_time, time_for_rest_points)
    print(f'Traj5 -> x: {traj5[0]} | y: {traj5[1]} | z: {traj5[2]}')
    plot_trajectories(traj1,  initial_time, time_for_p0_to_p1)


if __name__ == '__main__':
    main()
