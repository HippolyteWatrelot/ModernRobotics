import numpy as np
import modern_robotics as mr
import csv

M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67]
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]

"""arbitrary thetalist"""
thetalist1 = np.array([0, 0, 0, 0, 0, 0], dtype='float64')                                                              # Home configuration: Simulation1
thetalist2 = np.array([0, -1, 0, 0, 0, 0], dtype='float64')                                                             # Other configuration: Simulation2
dthetalist1 = np.zeros(6)
dthetalist2 = np.zeros(6)
Ftip = np.zeros(6)                                                                                                      # No force on end effector
g = np.array([0, 0, -9.81])
taulist = np.zeros(6)                                                                                                   # No torques

#print(mr.ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist))


def movement(thetalist, dthetalist, g, Ftip, Mlist, Glist, Slist):

    thetalists = [np.array(thetalist)]
    for i in range(500):                                                                                                # Let's do 500 steps (5 seconds video with 100 steps per second)
        ddthetalist = mr.ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist)                  # forward dynamics for actual configuration
        current_dthetalist = dthetalist                                                                                 # Saving current thetalist in another variable
        dthetalist += 0.01 * ddthetalist                                                                                # dthetalist update considering ddthetalist ~ constant for a short step, DeltaT = 0.01 sec
        mean_dthetalist = (current_dthetalist + dthetalist) / 2                                                         # Computing the mean dthetalist during a single step
        thetalist += 0.01 * mean_dthetalist                                                                             # thetalist update
        thetalists.append(np.array(thetalist))                                                                          # Adding new thetalist in Tab
    return np.array(thetalists)


thetalists_1 = movement(thetalist1, dthetalist1, g, Ftip, Mlist, Glist, Slist)

with open('simulation1.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    for i in range(len(thetalists_1)):
        writer.writerow([thetalists_1[i, j] for j in range(len(thetalists_1[i]))])                                      # CSV file for simulation 1

thetalists_2 = movement(thetalist2, dthetalist2, g, Ftip, Mlist, Glist, Slist)

with open('simulation2.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    for i in range(len(thetalists_2)):
        writer.writerow([thetalists_2[i, j] for j in range(len(thetalists_2[i]))])                                      # CSV file for simulation 2