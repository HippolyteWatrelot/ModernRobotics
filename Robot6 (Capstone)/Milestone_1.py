import modern_robotics as mr
import numpy as np
from math import atan2


def NextState(config, var_vec, max_ang_speed=1000000, r=0.0475, l=0.235, w=0.15, delta_t=0.01):

    """ - config: chassis (phi, x, y) position, full wheels and joints angles and gripper state configuration
        - var_vec: wheels and joints variation at one step"""

    assert len(config) == 13
    assert len(var_vec) == 9
    delta_theta = np.zeros(4)
    phi, x, y = config[:3]
    Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                    [np.sin(phi), np.cos(phi), 0, y],
                    [0, 0, 1, 0.0963],
                    [0, 0, 0, 1]])
    for k in range(4):
        delta_theta[k] = var_vec[k] * delta_t
    for i in range(9):
        assert abs(var_vec[i]) <= max_ang_speed
        config[i + 3] += delta_t * var_vec[i]
    F = (r / 4) * np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
                            [1, 1, 1, 1],
                            [-1, 1, -1, 1]])
    Vb = F @ delta_theta / delta_t
    Vb6 = np.zeros(6)
    for n in range(len(Vb)):
        Vb6[2 + n] = Vb[n]
    Vb6_brackets = mr.VecTose3(Vb6)
    Tbbnew = mr.MatrixExp6(Vb6_brackets)
    Tsbnew = Tsb @ Tbbnew
    new_phi = atan2(Tsbnew[1, 0], Tsbnew[0, 0])
    new_x = Tsbnew[0, 3]
    new_y = Tsbnew[1, 3]
    config[0] = new_phi
    config[1] = new_x
    config[2] = new_y

    return config


#config = [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
#var_vec9 = [-10, 10, 10, -10, np.pi/4, np.pi/4, np.pi/4, np.pi/4, np.pi/4]

#print('new state:\n', NextState(config, var_vec9))

