from random import uniform
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image

"""In this file, we simply want to place a certain number of nodes in the wanted space.
These nodes define the different spots of the graph for the agent"""

x = []
y = []
dots = []
for i in range(10):
    x = uniform(-0.5, 0.5)
    y = uniform(-0.5, 0.5)
    dots.append([x, y])
print(dots)

"""The matrixing function is only there to show a (discretized) picture of the placed dots
in the space, it is not important for the full program"""

def matrixing(dots, n):
    A = np.zeros([n, n])
    for i in range(len(dots)):
        a = round((n-1)*(dots[i][0] + 0.5), 0)
        b = round((n-1)*(dots[i][1] + 0.5), 0)
        #print("(a, b) = ", a, b)
        A[int(a), int(b)] = 1
    return np.rot90(np.rot90(np.rot90(A)))

#print(matrixing(dots, 15))


"""The "ensemble" function is there to dispatch these nodes 
"(Taking the obstacles into account)"""
"""-> Note that in the "cylinders2" file, I slightly change the 
diameter of each cylinders as I did take the robot dimensions into account"""


def ensemble(n):
    f = open("cylinders2", "r")
    g = f.read()
    file = g.splitlines()
    f.close()
    A = []
    i = 0
    while i < n:
        a = round(uniform(-0.5, 0.5), 2)
        b = round(uniform(-0.5, 0.5), 2)
        k = 0
        for j in range(len(file)):
            x1 = float(file[j].split()[0])
            y1 = float(file[j].split()[1])
            r1 = float(file[j].split()[2]) / 2
            if (a - x1) ** 2 + (b - y1) ** 2 > r1 ** 2:
                k += 1
        if k == len(file):
            A.append([a, b])
            i += 1
    return A

#print(matrixing(ensemble(10000), 24))
#pic = Image.fromarray(256*matrixing(ensemble(60000), 300))
#print(pic)
#plt.imshow(pic)
#plt.show()

n = 10
"""n is number of nodes excepting start and end dots"""
f = open("ScatteredNodes2", "w")
"""This file will let the coordinates of the dispatched dots appear"""
a = ensemble(n)
pic = Image.fromarray(256*matrixing(a, 100))
"""Here, I just wanted to know what did the dispatch look like"""
print(pic)
plt.imshow(pic)
plt.show()
start = [-0.5, -0.5]
end = [0.5, 0.5]
print(start[0], start[1], file=f)
for i in range(n):
    print(a[i][0], a[i][1], file=f)
print(end[0], end[1], file=f)
f.close()



