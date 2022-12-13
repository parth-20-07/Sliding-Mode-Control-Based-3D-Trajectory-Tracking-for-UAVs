#!/usr/bin/env python3
import matplotlib.pyplot as plt
import pickle
from mpl_toolkits.mplot3d import Axes3D


def visualization(x_series, y_series, z_series, traj_x_series, traj_y_series, traj_z_series):
    # load csv file and plot trajectory
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    # Data for a three-dimensional line
    ax.plot3D(x_series, y_series, z_series, 'blue')
    ax.plot3D(traj_x_series, traj_y_series, traj_z_series, 'red')
    plt.xlim(-0.5, 1.5)
    plt.ylim(-0.5, 1.5)
    plt.minorticks_on()
    plt.grid(which='both')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.savefig('trajectory.png', dpi=300)
    plt.show()


if __name__ == '__main__':
    file = open("log.pkl", 'rb')
    t_series, x_series, y_series, z_series, traj_x_series, traj_y_series, traj_z_series = pickle.load(
        file)
    file.close()
    visualization(x_series, y_series, z_series,
                  traj_x_series, traj_y_series, traj_z_series)
