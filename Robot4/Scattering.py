from random import uniform
import numpy as np
from math import sqrt
from pyinterval import interval as it
from matplotlib import pyplot as plt
from PIL import Image


x = []
y = []
dots = []
for i in range(10):
    x = uniform(-0.5, 0.5)
    y = uniform(-0.5, 0.5)
    dots.append([x, y])
print(dots)

def matrixing(dots, n):
    A = np.zeros([n, n])
    for i in range(len(dots)):
        a = round((n-1)*(dots[i][0] + 0.5), 0)
        b = round((n-1)*(dots[i][1] + 0.5), 0)
        #print("(a, b) = ", a, b)
        A[int(a), int(b)] = 1
    return np.rot90(np.rot90(np.rot90(A)))

#print(matrixing(dots, 15))



def ensemble(n):
    f = open("cylinders", "r")
    g = f.read()
    file = g.splitlines()
    f.close()
    A = []
    i = 0
    while i < n:
        a = uniform(-0.5, 0.5)
        b = uniform(-0.5, 0.5)
        k = 0
        for j in range(len(file)):
            x1 = float(file[j].split()[0])
            y1 = float(file[j].split()[1])
            r1 = float(file[j].split()[2]) / 2
            if r1 ** 2 > (b - y1) ** 2:
                if a not in it.interval[x1 - sqrt(r1 ** 2 - (b - y1) ** 2), x1 + sqrt(r1 ** 2 - (b - y1) ** 2)]:
                    k += 1
            else:
                if r1 ** 2 == (b - y1) ** 2:
                    if a != x1:
                        k += 1
                else:
                    if r1 ** 2 < (b - y1) ** 2:
                        k += 1
        if k == len(file):
            A.append([a, b])
            i += 1
    return A

#print(matrixing(ensemble(10000), 24))
pic = Image.fromarray(256*matrixing(ensemble(300000), 1000))
#print(pic)
plt.imshow(pic)
plt.show()

#n = 10
#f = open("ScatteredDots", "w")
#start = [-0.5, -0.5]
#end = [0.5, 0.5]
#print(start[0], start[1], file=f)
#for i in range(n):
#    print(ensemble(n)[i][0], ensemble(n)[i][1], file=f)
#print(end[0], end[1], file=f)
#f.close()



