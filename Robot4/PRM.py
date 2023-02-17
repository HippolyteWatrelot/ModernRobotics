import numpy as np
from math import sqrt, nan, isnan
from pyinterval import interval as it
from sympy.solvers import solve_univariate_inequality
from sympy import Symbol
from pandas import DataFrame as df


"""This function simply report all the distances from one node to another"""

def distances(file):
    F = open(file, "r")
    f1 = F.read()
    f = f1.splitlines()
    F.close()
    tab = np.zeros([len(f), len(f)])
    for i in range(len(f) - 1):
        for j in range(i + 1, len(f)):
            tab[i, j] = tab[j, i] = sqrt((float(f[j].split()[0]) - float(f[i].split()[0])) ** 2 + (float(f[j].split()[1]) - float(f[i].split()[1])) ** 2)
    return tab

#print(distances("ScatteredDots"))



"Probabilistic road map: apply the k nearest neighbors for all the scattered dots"

def PRM(distances, file, k):

    F = open(file, "r")
    f1 = F.read()
    f = f1.splitlines()
    F.close()
    N = nan * np.ones([len(f), k], dtype=int)
    tab1 = np.zeros(k)
    tab2 = np.zeros(k)

    for i in range(len(f)):
        group = []
        print('i', i)
        for s in range(len(f)):
            group.append((s, distances[i, s]))
        print(group)                                                                                                    #an array grouping all the nodes and their respective distances to node i
        dtype = [('s', int), ('distance', float)]
        a = np.array(group, dtype=dtype)
        group2 = np.sort(a, order='distance')
        print(group2)                                                                                                   #the rearanged labeled nodes in order to have the distances from node i from the smallest to the largest
        SortedDistances = np.sort(distances[i])
        SortedS = []
        for t in range(k):
            SortedS.append(group2[t+1][0])
        for j in range(k):
            tab1[j] = SortedDistances[j]
            tab2[j] = SortedS[j]
            x = Symbol('x')
            X = x * float(f[i].split()[0]) + (1 - x) * float(f[j].split()[0])
            Y = x * float(f[i].split()[1]) + (1 - x) * float(f[j].split()[1])
            g = open("cylinders", "r")
            g1 = g.read()
            file = g1.splitlines()
            g.close()
            for n in range(len(file)):
                x1 = float(file[n].split()[0])
                y1 = float(file[n].split()[1])
                r1 = float(file[n].split()[2]) / 2
                if solve_univariate_inequality((X - x1) ** 2 + (Y - y1) ** 2 < r1, x) == False:                         #The coordinetes(X,Y) mustn't be (whatever x in [0, 1] would be) in the circles defined by the cylinders
                    N[i, j] = int(tab2[j] + 1)                                                                          #enumerate the nearest nodes
    return N


distances = distances('ScatteredDots')
k = 5
A = PRM(distances, 'ScatteredDots', k)
L = A.shape[0]
print(A)
f = open("neighborhood", "w")
g = open("neighborhood(edges)", "w")
h = open("AllLinks", "w")
for i in range(L):
    print(A[i, :k], file=f)
    R = []
    for j in range(k):
        if not isnan(A[i, j]):
            R.append(distances[i, int(A[i, j] - 1)])
    print(R, file=g)
B = np.zeros([L, L - 1])
for i in range(L):
    for j in range(k):
        B[i, j] = int(A[i, j])
        for n in range(k, A.shape[0] - 1):
            B[i, n] = nan
            for s in range(L):
                if A[s, j] - 1 == i and s != i and s + 1 not in B[i]:
                    B[i, n] = s + 1
    print(B[i], file=h)
f.close()
g.close()
h.close()

y = open("AllLinksEdges", "w")
print(B)
C = nan * np.ones([L, L - 1])
for i in range(L):
    for j in range(L - 1):
        if not isnan(B[i, j]):
            C[i, j] = distances[i, int(B[i, j] - 1)]
    print(C[i], file=y)
y.close()
