import modern_robotics as mr
import numpy as np


def Feedback_Control(Tse, Tsed, Kp, Ki, Int_error, delta_t=0.01):

    Mat_Xerr = mr.MatrixLog6(mr.TransInv(Tse) @ Tsed)
    Xerr = mr.se3ToVec(Mat_Xerr)
    Int_error += Xerr * delta_t
    V = Kp @ Xerr + Ki @ Int_error

    return V, Xerr, Int_error


def FeedForward_Control(Tse, Tsed, TsedNext, Kp, Ki, Int_error, delta_t=0.01):

    Teed = mr.TransInv(Tse) @ Tsed
    Mat_Xerr = mr.MatrixLog6(Teed)
    Xerr = mr.se3ToVec(Mat_Xerr)
    Mat_Vd = mr.MatrixLog6(mr.TransInv(Tsed) @ TsedNext) / delta_t
    Vd = mr.se3ToVec(Mat_Vd)
    Ad_Teed = mr.Adjoint(Teed)
    Int_error += Xerr * delta_t
    V = Ad_Teed @ Vd + Kp @ Xerr + Ki @ Int_error
    # print("Xerr", Xerr)
    # print("Vd", Vd)
    # print("Ad_Mat @ Vd", Ad_Teed @ Vd)
    # print("V", V)

    return V, Xerr, Int_error


"""These lines are for testing: all is all right according to the documentation"""
#theta = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
#Blist = np.array([[0, 0, 1, 0, 0.033, 0],
#                  [0, -1, 0, -0.5076, 0, 0],
#                  [0, -1, 0, -0.3526, 0, 0],
#                  [0, -1, 0, -0.2176, 0, 0],
#                  [0, 0, 1, 0, 0, 0]]).T
#thetalist = theta[3:]
#Tb0 = np.array([[1, 0, 0, 0.1662],
#                [0, 1, 0, 0],
#                [0, 0, 1, 0.0026],
#                [0, 0, 0, 1]])
#M0e = np.array([[1, 0, 0, 0.033],
#                [0, 1, 0, 0],
#                [0, 0, 1, 0.6546],
#                [0, 0, 0, 1]])
#T0e = mr.FKinBody(M0e, Blist, thetalist)
#Tsb = np.array([[1, 0, 0, 0],
#                [0, 1, 0, 0],
#                [0, 0, 1, 0.0963],
#                [0, 0, 0, 1]])

#Tse = Tsb @ (Tb0 @ T0e)
#Tsed = np.array([[0, 0, 1, 0.5],
#                [0, 1, 0, 0],
#                [-1, 0, 0, 0.5],
#                [0, 0, 0, 1]])
#TsedNext = np.array([[0, 0, 1, 0.6],
#                     [0, 1, 0, 0],
#                     [-1, 0, 0, 0.3],
#                     [0, 0, 0, 1]])
#Kp, Ki = np.zeros((6, 6)), np.zeros((6, 6))
#Kp, Ki = np.diag(np.ones(6)), np.zeros((6, 6))
#V = FeedForward_Control(Tse, Tsed, TsedNext, Kp, Ki)[0]

#print("thetalist", thetalist)
#Arm_Jacobian = mr.JacobianBody(Blist, thetalist)
#r, l, w = 0.0475, 0.235, 0.15
#F = (r / 4) * np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
#                        [1, 1, 1, 1],
#                        [-1, 1, -1, 1]])
#M1 = np.zeros((2, 4))
#M2 = np.zeros((1, 4))
#F = np.concatenate((M1, F), axis=0)
#F6 = np.concatenate((F, M2), axis=0)
#Teb = mr.TransInv(T0e) @ mr.TransInv(Tb0)
#Base_Jacobian = mr.Adjoint(Teb) @ F6
#Jacobian = np.concatenate((Base_Jacobian, Arm_Jacobian), axis=1)
#print("Je\n", Jacobian, "\n")
#JacobianCross = np.linalg.pinv(Jacobian)
#u_theta_dot = JacobianCross @ V

#print("u_theta_dot\n", u_theta_dot, "\n")



"""Don't pay attention to this function"""
def Decoupled_Control(Tse, Tsed, TsedNext, Kp, Ki, delta_t=0.01):

    R, Rd, RdNext = Tse[:3, :3], Tsed[3:, 3:], TsedNext[:3, :3]
    we_bracket = mr.MatrixLog3(R.T @ Rd)
    we = mr.so3ToVec(we_bracket)
    wd_bracket = mr.MatrixLog3(mr.RotInv(Rd) @ RdNext)
    wd = mr.so3ToVec(wd_bracket)
    pd_next, pd = RdNext[:, 3], Rd[:, 3]
    pd_dot = (pd_next - pd) / delta_t
    Xerr = np.zeros(6)
    Vec_d = np.zeros(6)
    Mat = np.diag([0, 0, 0, 1, 1, 1])
    for i in range(3):
        Xerr[i] = we[i]
        Xerr[i + 3] = Tsed[i, 3] - Tse[i, 3]
        Vec_d[i] = wd[i]
        Vec_d[3 + i] = pd_dot[i]
        for j in range(3):
            Mat[i, j] = we_bracket[i, j]
    Vec = Mat @ Vec_d + Kp @ Xerr + Ki @ Xerr * delta_t
    print("Vec", Vec)

    return Vec



