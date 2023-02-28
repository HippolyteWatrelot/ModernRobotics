import modern_robotics as mr
import numpy as np
import csv


def RTG_decoupled(Tse, Tgoal, Trajectory, k, time, gripper_state):

    """When this function is called, Tse = TscStandoff.
       No rotations here"""

    pos_init = np.array([Tse[0, 3], Tse[1, 3], Tse[2, 3]])
    pos_final = np.array([Tgoal[0, 3], Tgoal[1, 3], Tgoal[2, 3]])
    Rse = Tse[:3, :3]
    ps = []
    for i in range(time * k * 100):
        s = (i + 1) / (time * k * 100)
        ps = pos_init + s * (pos_final - pos_init)
        Trajectory.append([Rse[0, 0], Rse[0, 1], Rse[0, 2],
                           Rse[1, 0], Rse[1, 1], Rse[1, 2],
                           Rse[2, 0], Rse[2, 1], Rse[2, 2],
                           ps[0], ps[1], ps[2], gripper_state])
    Tse[0, 3], Tse[1, 3], Tse[2, 3] = ps[0], ps[1], ps[2]
    print(Tse)
    return Tse, Trajectory

def RTG_decoupled_3(Tse, Tgoal, Trajectory, k, time, gripper_state):

    """When this function is called, Tse = TscStandoff.
       No rotations here"""

    pos_init = np.array([Tse[0, 3], Tse[1, 3], Tse[2, 3]])
    pos_final = np.array([Tgoal[0, 3], Tgoal[1, 3], Tgoal[2, 3]])
    Rse = Tse[:3, :3]
    ps = []
    for i in range(time * k * 100):
        T = time * k * 100
        a2, a3 = 3 / (T ** 2), -2 / (T ** 3)
        s = a2 * (i + 1) ** 2 + a3 * (i + 1) ** 3
        ps = pos_init + s * (pos_final - pos_init)
        Trajectory.append([Rse[0, 0], Rse[0, 1], Rse[0, 2],
                           Rse[1, 0], Rse[1, 1], Rse[1, 2],
                           Rse[2, 0], Rse[2, 1], Rse[2, 2],
                           ps[0], ps[1], ps[2], gripper_state])
    Tse[0, 3], Tse[1, 3], Tse[2, 3] = ps[0], ps[1], ps[2]
    print(Tse)
    return Tse, Trajectory


def RTG_decoupled_5(Tse, Tgoal, Trajectory, k, time, gripper_state):

    """When this function is called, Tse = TscStandoff.
       No rotations here as it is called for lowering and rising end effector for grasping and ungrasping"""

    pos_init = np.array([Tse[0, 3], Tse[1, 3], Tse[2, 3]])
    pos_final = np.array([Tgoal[0, 3], Tgoal[1, 3], Tgoal[2, 3]])
    Rse = Tse[:3, :3]
    ps = []
    for i in range(time * k * 100):
        T = time * k * 100
        a3, a4, a5 = 10 / (T ** 3), -15 / (T ** 4), 6 / (T ** 5)
        s = a3 * (i + 1) ** 3 + a4 * (i + 1) ** 4 + a5 * (i + 1) ** 5
        ps = pos_init + s * (pos_final - pos_init)
        Trajectory.append([Rse[0, 0], Rse[0, 1], Rse[0, 2],
                           Rse[1, 0], Rse[1, 1], Rse[1, 2],
                           Rse[2, 0], Rse[2, 1], Rse[2, 2],
                           ps[0], ps[1], ps[2], gripper_state])
    Tse[0, 3], Tse[1, 3], Tse[2, 3] = ps[0], ps[1], ps[2]
    print(Tse)
    return Tse, Trajectory


def RTG_Screw_path(Tse, Tgoal, Trajectory, k, time, gripper_state):

    Ts = []
    for i in range(time * k * 100):
        s = (i + 1) / (time * k * 100)
        Ts = Tse @ mr.MatrixExp6(mr.MatrixLog6(mr.TransInv(Tse) @ Tgoal) * s)
        Trajectory.append([Ts[0, 0], Ts[0, 1], Ts[0, 2],
                           Ts[1, 0], Ts[1, 1], Ts[1, 2],
                           Ts[2, 0], Ts[2, 1], Ts[2, 2],
                           Ts[0, 3], Ts[1, 3], Ts[2, 3], gripper_state])
    Tse = Ts
    return Tse, Trajectory


def RTG_Poly3_path(Tse, Tgoal, Trajectory, k, time, gripper_state):

    Ts = []
    for i in range(time * k * 100):
        T = time * k * 100
        a2, a3 = 3 / (T ** 2), -2 / (T ** 3)
        s = a2 * (i + 1) ** 2 + a3 * (i + 1) ** 3
        Ts = Tse @ mr.MatrixExp6(mr.MatrixLog6(mr.TransInv(Tse) @ Tgoal) * s)
        Trajectory.append([Ts[0, 0], Ts[0, 1], Ts[0, 2],
                           Ts[1, 0], Ts[1, 1], Ts[1, 2],
                           Ts[2, 0], Ts[2, 1], Ts[2, 2],
                           Ts[0, 3], Ts[1, 3], Ts[2, 3], gripper_state])
    Tse = Ts

    return Tse, Trajectory

def RTG_Poly5_path(Tse, Tgoal, Trajectory, k, time, gripper_state):

    Ts = []
    for i in range(time * k * 100):
        T = time * k * 100
        a3, a4, a5 = 10 / (T ** 3), -15 / (T ** 4), 6 / (T ** 5)
        s = a3 * (i + 1) ** 3 + a4 * (i + 1) ** 4 + a5 * (i + 1) ** 5
        Ts = Tse @ mr.MatrixExp6(mr.MatrixLog6(mr.TransInv(Tse) @ Tgoal) * s)
        Trajectory.append([Ts[0, 0], Ts[0, 1], Ts[0, 2],
                           Ts[1, 0], Ts[1, 1], Ts[1, 2],
                           Ts[2, 0], Ts[2, 1], Ts[2, 2],
                           Ts[0, 3], Ts[1, 3], Ts[2, 3], gripper_state])
    Tse = Ts

    return Tse, Trajectory


def Full_Trajectory(Tse, Tsc, TscFinal, TceGrasp, TceStandoff, k=1, time=30):

    time = int(time // 6)              # 6 portions of trajectory
    #Trajectory = [[Tse[0, 0], Tse[0, 1], Tse[0, 2], Tse[1, 0], Tse[1, 1], Tse[1, 2], Tse[2, 0], Tse[2, 1], Tse[2, 2],
    #               Tse[0, 3], Tse[1, 3], Tse[2, 3], 0]]
    Trajectory = []
    gripper_state = 0
    """Step 1"""
    TseStandoff = Tsc @ TceStandoff
    Tse, Trajectory = RTG_Screw_path(Tse, TseStandoff, Trajectory, k, time, gripper_state)
    #Tse, Trajectory = RTG_Poly5_path(Tse, TseStandoff, Trajectory, k, time, gripper_state)
    """Step 2"""
    TseGrasp = Tsc @ TceGrasp
    Tse, Trajectory = RTG_decoupled(Tse, TseGrasp, Trajectory, k, time, gripper_state)
    #Tse, Trajectory = RTG_Screw_path(Tse, TseGrasp, Trajectory, k, time, gripper_state)
    #Tse, Trajectory = RTG_decoupled_5(Tse, TseGrasp, Trajectory, k, time, gripper_state)
    """Step 3"""
    gripper_state = 1
    """Step 4"""
    Tse, Trajectory = RTG_decoupled(Tse, TseStandoff, Trajectory, k, time, gripper_state)
    #Tse, Trajectory = RTG_decoupled_5(Tse, TseStandoff, Trajectory, k, time, gripper_state)
    """Step 5"""
    TseFinalStandoff = TscFinal @ TceStandoff
    print('TseFinalStandoff\n', TseFinalStandoff)
    Tse, Trajectory = RTG_Screw_path(Tse, TseFinalStandoff, Trajectory, k, time, gripper_state)
    #Tse, Trajectory = RTG_Poly5_path(Tse, TseFinalStandoff, Trajectory, k, time, gripper_state)
    """Step 6"""
    #pos_Grasp = TccFinal @ np.array([TseGrasp[0, 3], TseGrasp[1, 3], TseGrasp[2, 3], 1])
    TseFinalGrasp = TscFinal @ TceGrasp
    Tse, Trajectory = RTG_decoupled(Tse, TseFinalGrasp, Trajectory, k, time, gripper_state)
    #Tse, Trajectory = RTG_decoupled_5(Tse, TseFinalGrasp, Trajectory, k, time, gripper_state)
    """Step 7"""
    gripper_state = 0
    """Step 8"""
    Tse, Trajectory = RTG_decoupled(Tse, TseFinalStandoff, Trajectory, k, time, gripper_state)
    #Tse, Trajectory = RTG_decoupled_5(Tse, TseFinalStandoff, Trajectory, k, time, gripper_state)

    return Trajectory


CP = np.array([1, 0, 0])
Tsc = np.array([[np.cos(CP[2]), -np.sin(CP[2]), 0, CP[0]],
                [np.sin(CP[2]), np.cos(CP[2]), 0, CP[1]],
                [0, 0, 1, 0.025],
                [0, 0, 0, 1]])
Tsed = np.array([[0, 0, 1, 0],
                [0, 1, 0, 0],
                [-1, 0, 0, 0.5],
                [0, 0, 0, 1]])
FP = np.array([0, - 1, -np.pi / 2])
TscFinal = np.array([[np.cos(FP[2]), -np.sin(FP[2]), 0, FP[0]],
                     [np.sin(FP[2]), np.cos(FP[2]), 0, FP[1]],
                     [0, 0, 1, 0.025],
                     [0, 0, 0, 1]])
TceGrasp = np.array([[0, 0, 1, 0],
                     [0, 1, 0, 0],
                     [-1, 0, 1, 0],
                     [0, 0, 0, 1]])
TceStandoff = np.array([[0, 0, 1, 0],
                        [0, 1, 0, 0],
                        [-1, 0, 0, 0.2],
                        [0, 0, 0, 1]])

print("TseStandoff\n", Tsc @ TceStandoff)

Trajectory = Full_Trajectory(Tsed, Tsc, TscFinal, TceGrasp, TceStandoff)
with open('Trajectory.csv', 'w', newline='') as file1:
    writer1 = csv.writer(file1)
    for i in range(len(Trajectory)):
        writer1.writerow([Trajectory[i][0], Trajectory[i][1], Trajectory[i][2], Trajectory[i][3], Trajectory[i][4],
                          Trajectory[i][5], Trajectory[i][6], Trajectory[i][7], Trajectory[i][8], Trajectory[i][9],
                          Trajectory[i][10], Trajectory[i][11], Trajectory[i][12]])