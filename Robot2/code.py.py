import modern_robotics as mr
import numpy as np
import csv

Tsd = np.array([[0, 1, 0, -0.5], [0, 0, -1, 0.1], [-1, 0, 0, 0.1], [0, 0, 0, 1]])                                       # Desired T
pre_thetalist = np.array([-0.14094739, -2.18864901, -1.22948707, -2.61878742, -3.11228918, -1.32504376])
thetalist = (pre_thetalist + np.pi) % (2 * np.pi) - np.pi                                                               # Writing angles in [-pi, pi]
#thetalist_deg = 180 * thetalist / np.pi
#print("thetalist0", thetalist_deg)
M = np.array([[-1, 0, 0, 0.817], [0, 0, 1, 0.191], [0, 1, 0, -0.06], [0, 0, 0, 1]])
Blist = np.array([[0, 1, 0, 0.191, 0, 0.817], [0, 0, 1, 0.095, -0.817, 0], [0, 0, 1, 0.095, -0.392, 0], [0, 0, 1, 0.095, 0, 0], [0, -1, 0, -0.082, 0, 0], [0, 0, 1, 0, 0, 0]]).T
eomg = 0.001
ev = 0.0001

#goal_TL = mr.IKinBody(Blist, M, Tsd, thetalist, eomg, ev)
#help(mr.IKinBody)
#print(mr.IKinBody(Blist, M, Tsd, thetalist, eomg, ev))


def IKinBodyIterates(Blist, M, T, thetalist, eomg, ev):

    joints_vectors = []
    vals = thetalist
    joints_vectors.append(np.array(vals))
    Tsb = mr.FKinBody(M, Blist, thetalist)
    Inv_Tsb = np.linalg.inv(Tsb)
    Mat_Vb = mr.MatrixLog6(Inv_Tsb @ T)                                                                                 # Vb in its matrix form
    Vb = mr.se3ToVec(Mat_Vb)
    aem = np.sqrt(Vb[0] ** 2 + Vb[1] ** 2 + Vb[2] ** 2)                                                                 # Angular error Magnitude
    lem = np.sqrt(Vb[3] ** 2 + Vb[4] ** 2 + Vb[5] ** 2)                                                                 # Linear error Magnitude
    print("joint vector", thetalist)
    print("SE(3) end effector config", Tsb)
    print("error twist", Vb)
    print("angular error magnitude", aem)
    print("linear error magnitude", lem)
    print()
    while aem > eomg or lem > ev:
        Jb = mr.JacobianBody(Blist, thetalist)                                                                          # Jacobian Body
        PI_Jb = np.linalg.inv(Jb.T @ Jb) @ Jb.T                                                                         # Pseudo_Inverse Jacobian Body
        thetalist += PI_Jb @ Vb
        for i in range(len(thetalist)):
            thetalist[i] = (thetalist[i] + np.pi) % (2 * np.pi) - np.pi
        vals = thetalist
        joints_vectors.append(np.array(vals))
        Tsb = mr.FKinBody(M, Blist, thetalist)
        Inv_Tsb = np.linalg.inv(Tsb)
        Mat_Vb = mr.MatrixLog6(Inv_Tsb @ T)                                                                             # Vb in its matrix form
        Vb = mr.se3ToVec(Mat_Vb)
        aem = np.sqrt(Vb[0] ** 2 + Vb[1] ** 2 + Vb[2] ** 2)
        lem = np.sqrt(Vb[3] ** 2 + Vb[4] ** 2 + Vb[5] ** 2)
        print("joint vector:\n", thetalist)
        print("SE(3) end effector config:\n", Tsb)
        print("error twist:\n", Vb)
        print("angular error magnitude:", aem)
        print("linear error magnitude:", lem)
        print()

    return thetalist, np.array(joints_vectors)                                                                          # Desired thetalist and list of intermediate thetalists


outputs = IKinBodyIterates(Blist, M, Tsd, thetalist, eomg, ev)
new_thetalist = outputs[0]
new_thetalist_deg = 180 * new_thetalist / np.pi
Joint_Vectors = outputs[1]
print("final joint vector\n", new_thetalist)
print("final joint vector in deg\n", new_thetalist_deg)
print("All joints vectors\n", Joint_Vectors)
print("FINAL T\n", mr.FKinBody(M, Blist, new_thetalist))

with open('JV.csv', 'w', newline='') as file1:
    writer1 = csv.writer(file1)
    for i in range(len(Joint_Vectors)):
        writer1.writerow([Joint_Vectors[i, j] for j in range(len(Joint_Vectors[i]))])

