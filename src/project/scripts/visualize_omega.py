#!/usr/bin/env python3
import matplotlib.pyplot as plt
import pickle
from mpl_toolkits.mplot3d import Axes3D


def visualization(time, w1, w2, w3, w4):
    # load csv file and plot trajectory
    fig = plt.figure()

    plt.plot(time, w1, color='r', label='w1')
    plt.plot(time, w2, color='g', label='w2')
    plt.plot(time, w3, color='b', label='w3')
    plt.plot(time, w4, label='w4')

    plt.xlabel('time')
    plt.ylabel('omega(rad/s)')
    plt.title("Angular Velocity of Rotor")
    plt.savefig('omega.png', dpi=300)
    plt.legend()
    plt.show()


if __name__ == '__main__':
    file = open("omega_log.pkl", 'rb')
    time, w1, w2, w3, w4 = pickle.load(
        file)
    file.close()
    visualization(time, w1, w2, w3, w4)
