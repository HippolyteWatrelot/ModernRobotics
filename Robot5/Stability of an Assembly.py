import scipy.optimize as sp
import matplotlib.pyplot as plt
import numpy as np
from math import atan2, cos, sin, nan, sqrt
from sympy.solvers import solve
from sympy import Symbol
import Polygons

figure1 = [(0, 2), (0, 4), (2, 4), (2, 2), (0, 2)]
figure2 = [(1, 4), (1, 5), (3, 5), (3, 1), (1, 1), (1, 2), (2, 2), (2, 4), (1, 4)]
figure3 = [(0.5, 0), (0.5, 1), (2.5, 1), (2.5, 0), (0.5, 0)]

#figure1 = [(0, 2), (0, 3), (2, 3), (2, 2), (0, 2)]
#figure2 = [(1, 4), (1, 5), (3, 5), (3, 1), (1, 1), (1, 2), (2, 2), (2, 4), (1, 4)]
#figure3 = [(0, 0), (1.5, 1), (3, 0), (0, 0)]

figure1 = [(0, 1), (3, 4), (3, 3), (1, 1), (0, 1)]
figure2 = [(3, 3), (3, 4), (6, 1), (5, 1), (3, 3)]
figure3 = [(0, 0), (0, 1), (6, 1), (6, 0), (0, 0)]

x1, y1, x2, y2, x3, y3 = [], [], [], [], [], []

for i in range(len(figure1)):
    x1.append(figure1[i][0])
    y1.append(figure1[i][1])
for j in range(len(figure2)):
    x2.append(figure2[j][0])
    y2.append(figure2[j][1])
for h in range(len(figure3)):
    x3.append(figure3[h][0])
    y3.append(figure3[h][1])

X1, Y1, X2, Y2, X3, Y3 = np.array(x1), np.array(y1), np.array(x2), np.array(y2), np.array(x3), np.array(y3)
exe1, exe2, exe3 = Polygons.mass_and_center_of_gravity(figure1), Polygons.mass_and_center_of_gravity(figure2), \
                   Polygons.mass_and_center_of_gravity(figure3)
print()

"""mass1, mass2 and mass3 are the masses of each polygon of the assembly"""
mass1, mass2, mass3 = exe1[0], exe2[0], exe3[0]
print("figure1 mass", mass1)
print("figure2 mass", mass2)
print("figure3 mass", mass3)
print()

"""cog1, cog2 and cog3 are the centers of gravity of each polygon of the assembly, They will appear as red dots on plot 
for each polygon"""
cog1, cog2, cog3 = exe1[1], exe2[1], exe3[1]
print("figure1 center of gravity", cog1)
print("figure2 center of gravity", cog2)
print("figure3 center of gravity", cog3)
print()

"""This line just tests if we have a real assembly with the polygons (1 resulting block)"""
#assembly = Polygons.rigid_bodies_assembly([figure1, figure2, figure3])
#print()

"""CN returns the contact nodes coordinates and their associated contact vectors"""
CN = Polygons.contacts_vectors([figure1, figure2, figure3])
print()

"""These lines are here just for displaying the contact nodes as blue dots"""
contacts_nodes2 = CN[2]
for i in range(len(contacts_nodes2)):
    plt.plot([contacts_nodes2[i][0]], [contacts_nodes2[i][1]], 'bo')

"""Just plotting the polygons and their centers of gravity"""
plt.plot([cog1[0]], [cog1[1]], 'ro')
plt.plot([cog2[0]], [cog2[1]], 'ro')
plt.plot([cog3[0]], [cog3[1]], 'ro')
plt.plot(X1, Y1, "r")
plt.plot(X2, Y2, "g")
plt.plot(X3, Y3, "y")

"""Determination of contact vectors angles for each contact"""
contacts_nodes1 = CN[0]
contact_vectors_angles = CN[1]
print("contact nodes", contacts_nodes1)
print("contact vector angles (in radians)", contact_vectors_angles)
print()

"""These lines are just here to let the contact vectors appear without notion of scaling"""
for i in range(len(contacts_nodes1)):
    x_start = contacts_nodes1[i][0]
    x_end1 = x_start + cos(contact_vectors_angles[i][0]) * 0.2
    x_end2 = x_start + cos(contact_vectors_angles[i][1]) * 0.2
    y_start = contacts_nodes1[i][1]
    y_end1 = y_start + sin(contact_vectors_angles[i][0]) * 0.2
    y_end2 = y_start + sin(contact_vectors_angles[i][1]) * 0.2
    vecX1 = [x_start, x_end1]
    vecY1 = [y_start, y_end1]
    vecX2 = [x_start, x_end2]
    vecY2 = [y_start, y_end2]
    plt.plot(vecX1, vecY1, "black")
    plt.plot(vecX2, vecY2, "black")

plt.show()

g = (0, -9.81)
