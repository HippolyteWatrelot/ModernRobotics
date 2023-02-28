from Milestone_1 import NextState
from Milestone_3 import FeedForward_Control
import modern_robotics as mr
import numpy as np
import csv

'''Loading the full trajectory (Stack of vectors of dim 13)'''
with open('Trajectory.csv', newline='') as file:
    Trajectory = []
    for line in file.readlines():
        row = line.split(',')
        T = []
        for i in range(len(row)):
            T.append(float(row[i]))
        Trajectory.append(T)


'''Tsed is the original reference configuration of the en effector frame that has be used to generate
 the trajectory above'''
Tsed = np.array([[0, 0, 1, 0],
                 [0, 1, 0, 0],
                 [-1, 0, 0, 0.5],
                 [0, 0, 0, 1]])

'''Let's define the initial error for this end effector with an angle of 30° a z of -2 meters'''
Tede = np.array([[np.sqrt(3) / 2, 0, -0.5, 0],
                 [0, 1, 0, 0],
                 [0.5, 0, np.sqrt(3) / 2, -0.2],
                 [0, 0, 0, 1]])
#Tede = np.diag([1, 1, 1, 1])
Tse = Tsed @ Tede

Tb0 = np.array([[1, 0, 0, 0.1662],
                [0, 1, 0, 0],
                [0, 0, 1, 0.0026],
                [0, 0, 0, 1]])

'''Initial position for the chassis'''
x, y, phi = 0, 0, 0
Msb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                [np.sin(phi), np.cos(phi), 0, y],
                [0, 0, 1, 0.0963],
                [0, 0, 0, 1]])

T0e = mr.TransInv(Tb0) @ (mr.TransInv(Msb) @ Tse)

'''In this Blist, we consider the chassis's rotation as a joint, so we add it at the beginning
(It is a plus for the Newton-Raphson Algorithm)'''
Full_Blist = np.array([[0, 0, 0, 1, 0, 0],
                       [0, 0, 0, 0, 1, 0],
                       [0, 0, 1, 0, 0.1992, 0],
                       [0, 0, 1, 0, 0.033, 0],
                       [0, -1, 0, -0.5076, 0, 0],
                       [0, -1, 0, -0.3526, 0, 0],
                       [0, -1, 0, -0.2176, 0, 0],
                       [0, 0, 1, 0, 0, 0]]).T
Blist = np.array([[0, 0, 1, 0, 0.033, 0],
                  [0, -1, 0, -0.5076, 0, 0],
                  [0, -1, 0, -0.3526, 0, 0],
                  [0, -1, 0, -0.2176, 0, 0],
                  [0, 0, 1, 0, 0, 0]]).T
eomg = 0.001
ev = 0.0001
M0e = np.array([[1, 0, 0, 0.033],
                [0, 1, 0, 0],
                [0, 0, 1, 0.6546],
                [0, 0, 0, 1]])
Mbe = Tb0 @ M0e
Mse = Msb @ Mbe
print("Mse\n", Mse)
print('Tsed\n', Tsed)
print("Tse\n", Tse)
print()


'''Newton-Raphson Algorithm to determine a correct initial joints and wheels configuration for Tse.'''
Vb_bracket = mr.MatrixLog6(mr.TransInv(Mse) @ Tse)
Vb = mr.se3ToVec(Vb_bracket)
aem = np.sqrt(Vb[0] ** 2 + Vb[1] ** 2 + Vb[2] ** 2)
lem = np.sqrt(Vb[3] ** 2 + Vb[4] ** 2 + Vb[5] ** 2)
thetalist_e = np.zeros(8)
T = Mse
while aem > eomg or lem > ev:
    Jb = mr.JacobianBody(Full_Blist, thetalist_e)
    JbCross = np.linalg.pinv(Jb, 1e-4)
    thetalist_e += JbCross @ Vb
    T = mr.FKinBody(Mse, Full_Blist, thetalist_e)
    Vb_bracket = mr.MatrixLog6(mr.TransInv(T) @ Tse)
    Vb = mr.se3ToVec(Vb_bracket)
    aem = np.sqrt(Vb[0] ** 2 + Vb[1] ** 2 + Vb[2] ** 2)
    lem = np.sqrt(Vb[3] ** 2 + Vb[4] ** 2 + Vb[5] ** 2)

'''IMPORTANT: this thetalist describes the parameters in the following order:
   x, y, phi, joint1, joint2, joint3, joint4, joint5'''

#print("Full_thetalist_e\n", thetalist_e)
print("thetalist\n", thetalist_e)
print()
Blist = np.array([[0, 0, 1, 0, 0.033, 0],
                  [0, -1, 0, -0.5076, 0, 0],
                  [0, -1, 0, -0.3526, 0, 0],
                  [0, -1, 0, -0.2176, 0, 0],
                  [0, 0, 1, 0, 0, 0]]).T
x, y, phi = thetalist_e[:3]
Tsb = np.array([[1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0.0963],
                [0, 0, 0, 1]])
Tbbnew = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                   [np.sin(phi), np.cos(phi), 0, y],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
Tsbnew = Tsb @ Tbbnew
T0e = mr.FKinBody(M0e, Blist, thetalist_e[3:])
print("Found Tsb\n", Tsbnew)
print('Tse ?\n', mr.FKinBody(Mse, Full_Blist, thetalist_e))
print('Tse ?\n', Tsbnew @ (Tb0 @ T0e))
print()

r, l, w = 0.0475, 0.235, 0.15

"""No real need of these lines:"""
#H0 = (1 / r) * np.array([[-l - w, 1, -1],
#                         [l + w, 1, 1],
#                         [l + w, 1, -1],
#                         [-l - w, 1, 1]])
#'''Wheels configuration for error desired initial position'''
#x_e, y_e, phi_e = thetalist_e[:3]
#delta_q_e = np.array([[1, 0, 0],
#                      [0, np.cos(phi_e), np.sin(phi_e)],
#                      [0, -np.sin(phi_e), np.cos(phi_e)]])
#H_phi = H0 @ delta_q_e
#u_e = H_phi @ np.array([phi_e, x_e, y_e])
#print("Wheels config for initial error end effector\n", u_e, "\n")

"""Let's initialize wheels config at zero:"""
u_e = [0, 0, 0, 0]


init_config = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
init_config[0] = thetalist_e[2]
init_config[1:3] = thetalist_e[:2]
init_config[3:7] = u_e
init_config[7:12] = thetalist_e[3:8]
init_config[12] = 0
print("real init config\n", init_config, "\n")

F = (r / 4) * np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
                        [1, 1, 1, 1],
                        [-1, 1, -1, 1]])
print("r, l, w:", r, l, w, "\n")
M1 = np.zeros((2, 4))
M2 = np.zeros((1, 4))
pre_F6 = np.concatenate((M1, F), axis=0)
F6 = np.concatenate((pre_F6, M2), axis=0)
print("F6\n", F6, "\n")

k = 1
Kp = np.diag(k * np.ones(6))              # [1, 15] for overshoot with error
q = 15                                    # [5, 3] else
Ki = np.diag(q * np.ones(6))

Blist = np.array([[0, 0, 1, 0, 0.033, 0],
                  [0, -1, 0, -0.5076, 0, 0],
                  [0, -1, 0, -0.3526, 0, 0],
                  [0, -1, 0, -0.2176, 0, 0],
                  [0, 0, 1, 0, 0, 0]]).T

'''INITIALIZATION 1(We have already initialized T0e, Tsed and Tse)'''
"""MAIN LOOP: wheels and joints configurations for each step of the trajectory.csv file using feedforward control."""
Full_thetalist = thetalist_e
thetalist = Full_thetalist[3:]
config = init_config
print("init_e Full_Thetalist", Full_thetalist, "\n")
gripper_state = 0
Xerrs = []
Configs = np.zeros((int(len(Trajectory)), 13))
Configs[0][:] = config
delta_t = 0.01
Int_error = [0, 0, 0, 0, 0, 0]
"""Here I just wanted to keep the first portion of the trajectory (Robot moving to the object"""
for i in range(int(len(Trajectory)) - 1):
    M = Trajectory[i + 1]
    TsedNext = np.array([[M[0], M[1], M[2], M[9]],
                        [M[3], M[4], M[5], M[10]],
                        [M[6], M[7], M[8], M[11]],
                        [0, 0, 0, 1]])
    #actual_state = np.array([Tse[0, 0], Tse[0, 1], Tse[0, 2],
    #                         Tse[1, 0], Tse[1, 1], Tse[1, 2],
    #                         Tse[2, 0], Tse[2, 1], Tse[2, 2],
    #                         Tse[0, 3], Tse[1, 3], Tse[2, 3], gripper_state])
    Arm_Jacobian = mr.JacobianBody(Blist, thetalist)
    Teb = mr.TransInv(T0e) @ mr.TransInv(Tb0)
    Base_Jacobian = mr.Adjoint(Teb) @ F6
    Jacobian = np.concatenate((Base_Jacobian, Arm_Jacobian), axis=1)
    JacobianCross = np.linalg.pinv(Jacobian, 1e-3)
    X = FeedForward_Control(Tse, Tsed, TsedNext, Kp, Ki, Int_error)
    V = X[0]
    Xerr = X[1]
    Int_error = X[2]
    #print(i + 1, Xerr)
    Xerrs.append(Xerr)
    error_norm = np.sqrt(Xerr[0] ** 2 + Xerr[1] ** 2 + Xerr[2] ** 2 + Xerr[3] ** 2 + Xerr[4] ** 2 + Xerr[5] ** 2)
    print(i + 1, 'error', error_norm)
    u_theta = JacobianCross @ V
    '''Updating config (so its joints thetalist and chassis position (phi, x, y)),
    Tse and Tsed'''
    config = NextState(config, u_theta)
    C = np.array(config)
    gripper_state = M[12]
    config[12] = gripper_state
    thetalist = [config[7], config[8], config[9], config[10], config[11]]
    for j in range(len(thetalist)):
        thetalist[j] = thetalist[j] % (2 * np.pi)
    """New Arm T matrix:"""
    T0e = mr.FKinBody(M0e, Blist, thetalist)
    """New chassis config:"""
    phi, x, y = config[0], config[1], config[2]
    Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                    [np.sin(phi), np.cos(phi), 0, y],
                    [0, 0, 1, 0.0963],
                    [0, 0, 0, 1]])
    """New full robot T matrix"""
    Tse = Tsb @ (Tb0 @ T0e)
    #print('Tse\n', Tse)
    for j in range(13):
        Configs[i + 1, j] = C[j]
    Tsed = TsedNext

print('T0e\n', T0e)
print('Tsb\n', Tsb)
print('config', config)
print('Tse\n', Tse)
print('Tse bis\n', Tsb @ (Tb0 @ T0e))
#print('Tsed\n', Tsed)
#Mat_Xerr = mr.MatrixLog6(mr.TransInv(Tse) @ Tsed)
#print('Mat_Xerr', Mat_Xerr)
#print(mr.se3ToVec(Mat_Xerr))

'''Here we have our final Tse, Tsed, config and thetalist after the first part of trajectory'''

with open('Xerr.csv', 'w', newline='') as file1:
    writer1 = csv.writer(file1)
    for i in range(int(len(Trajectory)) - 1):
        writer1.writerow([Xerrs[i][0], Xerrs[i][1], Xerrs[i][2], Xerrs[i][3], Xerrs[i][4], Xerrs[i][5]])

with open('Configurations.csv', 'w', newline='') as file2:
    writer2 = csv.writer(file2)
    for i in range(int(len(Trajectory)) - 1):
        writer2.writerow([Configs[i][0], Configs[i][1], Configs[i][2], Configs[i][3], Configs[i][4], Configs[i][5],
                          Configs[i][6], Configs[i][7], Configs[i][8], Configs[i][9], Configs[i][10], Configs[i][11],
                          Configs[i][12]])