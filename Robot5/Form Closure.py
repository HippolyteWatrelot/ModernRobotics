import scipy.optimize as sp
import matplotlib.pyplot as plt
import numpy as np
import intervals as I
from math import atan2, cos, sin
from sympy.solvers import solve
from sympy import Symbol
from Polygons import IsPolygon, Contact_Placing


tab = [(0, 0), (0, 1), (1, 2), (2, 1), (0, 0)]
#tab = [(0, 0), (2, 1), (1, 2), (0, 1), (0, 0)]
POC = Contact_Placing(tab)
contacts = POC[0]
Normals = POC[1]
x1 = []
y1 = []
for i in range(len(tab)):
    x1.append(tab[i][0])
    y1.append(tab[i][1])
x = np.array(x1)
y = np.array(y1)
plt.plot(x, y)
for i in range(len(contacts)):
    x1 = contacts[i][0]
    y1 = contacts[i][1]
    angle = float(Normals[i])
    x2 = x1 + 0.1 * cos(angle)                                                                                          # Here, just a way to represent the normal vectors
    y2 = y1 + 0.1 * sin(angle)
    plt.plot([x1, x2], [y1, y2])
plt.show()
"""IMPORTANT: To verify the orthogonality for most of normal vectors, please verify 
that the two axis have same length for same dimensions (By adjusting the window)"""
print("contacts", contacts)
print("normal vector angles (in radians)", Normals)

F1 = []
cosinus = np.cos(Normals)
sinus = np.sin(Normals)
for i in range(len(Normals)):
    vector = contacts[i]
    if abs(cosinus[i]) < 1e-10:
        cosinus[i] = 0
    if abs(sinus[i]) < 1e-10:
        sinus[i] = 0
    w = -vector[1] * cosinus[i] + vector[0] * sinus[i]
    wrench = [w, cosinus[i], sinus[i]]
    F1.append(wrench)
F = np.array(F1).T

print("Wrenches Matrix:\n", F)
print()
c = np.zeros(len(contacts))
b = np.zeros(3)
k = 0
epsilon = 1e-5
"""Epsilon appears because of the imprecisions of the wrenches values and the linprog calculation.
We have to admit a threshold"""
if len(contacts) < 4:
    print("\nNot in form closure for first order (Not enough points of contact)")
    k = 1
else:
    if np.linalg.matrix_rank(F) == 3:
        X = sp.linprog(c, A_ub=None, b_ub=None, A_eq=F, b_eq=b, bounds=(0, None), method='interior-point')
        x = X['x']
        print("Evaluated solution vector:\n", x)
        for i in range(len(contacts)):
            if x[i] <= epsilon:
                """As said above, it should be written: x[i] <= 0, but cause of some imprecisions in the linear
                programming calculations, we have to consider a reliable threshold"""
                k = 1
    else:
        k = 1
    if k == 0:
        print("\nForm closure !")
    else:
        print("\nNot in form closure for first order")