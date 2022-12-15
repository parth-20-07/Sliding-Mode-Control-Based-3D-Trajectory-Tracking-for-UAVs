#!/usr/bin/env python3
import matplotlib.pyplot as plt
import pickle
from mpl_toolkits.mplot3d import Axes3D


def visualization(x_series, y_series, z_series, traj_x_series, traj_y_series, traj_z_series):
    # load csv file and plot trajectory
    fig = plt.figure()
    ax = plt.axes(projection='3d')

    # Data for a three-dimensional line
    ax.plot3D(x_series, y_series, z_series, color='r', label='actual path')
    ax.plot3D(traj_x_series, traj_y_series, traj_z_series,
              color='b', label='tajectory path')

    plt.grid(which='both')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.title("Drone Path")
    plt.savefig('src/project/scripts/trajectory.png', dpi=1200)
    plt.legend()
    plt.show()


if __name__ == '__main__':
    file = open("src/project/scripts/log.pkl", 'rb')
    t_series, x_series, y_series, z_series, traj_x_series, traj_y_series, traj_z_series = pickle.load(
        file)
    file.close()
    visualization(x_series, y_series, z_series,
                  traj_x_series, traj_y_series, traj_z_series)
