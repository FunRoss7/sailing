#!/usr/bin/env python3
import numpy as np
from numpy import sin, cos
from numpy.linalg import eig
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

def system(t, x):
    plant =  np.array([ [ 0.0, 1.0],
                        [-1, -0.1] ])
    desired_point = np.array([1.0,0.0])
    error = desired_point - x
    controller_gains = np.array([1,1])
    u = np.matmul(controller_gains, error)
    controller_actuation = np.array([0.0,1.0])
    return np.matmul(plant,x) + controller_actuation * u

if __name__ == "__main__":
    xi = np.array([ 1.0, 2.0])
    sol = solve_ivp(system, [0,50], xi, max_step=0.1)
    plt.plot(sol.t, sol.y[0])
    plt.show()