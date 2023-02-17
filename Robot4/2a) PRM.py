import numpy as np
from math import sqrt, nan, isnan
from sympy.solvers import solve_univariate_inequality
from sympy import Symbol


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

#print(distances("ScatteredNodes"))


"""Probabilistic road map: apply the k nearest neighbors for all scattered dots and make the edges if it's possible"""
""" (taking the cylinders obstacles into account)"""

def PRM(distances, k, file="ScatteredNodes"):

    """
       - f is simply the 'ScatteredNodes' file split in rows
       - tab1 and tab2 are tools to obtain the sorted nodes distances from each node
    """

    F = open(file, "r")
    f1 = F.read()
    f = f1.splitlines()
    F.close()
    N = nan * np.ones([len(f), k], dtype=int)
    tab1 = np.zeros(k)
    tab2 = np.zeros(k)

    for i in range(len(f)):
        group = []
        for s in range(len(f)):
            group.append((s, distances[i, s]))                                                                          #an array grouping all the nodes and their respective distances to node i
        dtype = [('s', int), ('distance', float)]
        a = np.array(group, dtype=dtype)
        group2 = np.sort(a, order='distance')                                                                           #the rearanged labeled nodes in order to have the distances from node i from the smallest to the largest
        SortedDistances = np.sort(distances[i])
        SortedLabels = []
        for t in range(k):
            SortedLabels.append(group2[t+1][0] + 1)
        for j in range(k):
            tab1[j] = SortedDistances[j]
            tab2[j] = SortedLabels[j]
            x = Symbol('x')
            y = 1 / (1 + x ** 2)
            X = y * float(f[i].split()[0]) + (1 - y) * float(f[int(tab2[j] - 1)].split()[0])
            Y = y * float(f[i].split()[1]) + (1 - y) * float(f[int(tab2[j] - 1)].split()[1])
            g = open("cylinders2", "r")
            g1 = g.read()
            file = g1.splitlines()
            g.close()
            d = 0
            for n in range(len(file)):
                x1 = float(file[n].split()[0])
                y1 = float(file[n].split()[1])
                r1 = float(file[n].split()[2]) / 2
                if solve_univariate_inequality((X - x1) ** 2 + (Y - y1) ** 2 < r1 ** 2, x) == False:                    #The coordinetes(X,Y) mustn't be (whatever x in [0, 1] would be) in the circles defined by the cylinders
                    d += 1
            if d == len(file):
                N[i, j] = int(tab2[j])
    D = nan * np.ones([len(f), k], dtype=int)                                                                           # D is just like N, however, we want to separate the linked neighbors labels from the not linked ones left and right
    for i in range(len(f)):
        for j in range(k):
            for s in range(k):
                if not isnan(N[i, s]) and int(N[i, s]) not in D[i]:
                    D[i, j] = N[i, s]
    return N, D


distances = distances('ScatteredNodes')
k = 12
A = PRM(distances, k)
C = A[1]
L = A[0].shape[0]
print("A", A[0])
print("C", C)
f = open("neighborhoodPRM", "w")                                                                                        #This file will contain the tab which for each row all the edgeable neighbors for the dot defined by this row
g = open("neighborhoodPRM(edges)", "w")                                                                                 #Same but this time, the neighbor dots labels will be replaced by the length of the edges separating the row dot and the neighbors dots

"""The next lines of this code are for showing each node with all their formed links.
Indeed, for each node, we know in their k first neighbors the linked ones, however, that does not mean that they are all
of their links: taking two nodes n1 and n2, if n2 is in the k nearest neighbors of n1 and is linked does not necessary
imply that n1 is in n2's k nearest neighbors."""

h = open("AllLinksPRM", "w")                                                                                            #The all links final tab will contain all the made links between dots thanks to the algorithms (listing all the edgeable neighbors is not sufficient)
for i in range(L):
    print(C[i, :k], file=f)
    R = []
    for j in range(k):
        if not isnan(C[i, j]):
            R.append(distances[i, int(C[i, j] - 1)])
    print(R, file=g)
B = np.zeros([L, L - 1])                                                                                                #Full tab initialization
for i in range(L):
    count = 0
    for j in range(k):
        if isnan(C[i, j]):
            count += 1
        if not isnan(C[i, j]):
            B[i, j] = int(C[i, j])
    for n in range(k - count, C.shape[0] - 1):
        B[i, n] = nan
    for j in range(k):
        for s in range(L):
            for n in range(k - count, C.shape[0] - 1):
                if C[s, j] - 1 == i and s != i and s + 1 not in B[i]:
                    B[i, n] = s + 1
    print(B[i], file=h)
f.close()
g.close()
h.close()

y = open("AllLinksEdgesPRM", "w")                                                                                       #Same but with distances this time
print(B)
C = nan * np.ones([L, L - 1])
for i in range(L):
    for j in range(L - 1):
        if not isnan(B[i, j]):
            C[i, j] = distances[i, int(B[i, j] - 1)]
    print(C[i], file=y)
y.close()
