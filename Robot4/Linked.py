import numpy as np
from math import nan, isnan, sqrt
from scipy.special import binom
import itertools as it

"""To know if there's at least one path from start point to end point after the k neighbors PRM (leading to n connections) in a logical point of view,
we simply have to verify that there are no isolated group of n dots including only one of the extreme dots (start or end) which have the same neighbors in the AllLinks file (including themselves) 
whatever the orders are. (n would be the maximum number of neighbors a dot could have in the group of n dots).
However, we now that neighbors can be 'nan', so we have to consider them, so I had to find a technique of comparison"""

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


f1 = open("AllLinks", "r")
F = f1.read()
f = F.splitlines()
L = len(f)
f[0] = f[0].translate({ord(r): None for r in '[].,'})
l = len(f[0].split())
k = 5
a = np.zeros(L)
tab = np.zeros([L, l])
labels = np.array([np.arange(L) + 1])
for i in range(L):
    f[i] = f[i].translate({ord(r): None for r in '[].,'})
    for j in range(len((f[i].split()))):
        if isnan(float(f[i].split()[j])):
            tab[i, j] = nan
        else:
            tab[i, j] = int(f[i].split()[j])
            a[i] += 1
tab = np.concatenate((labels.T, tab), 1)
#print(a)
print(tab)

distances = distances("ScatteredDots")

E = []
D = []
for i in range(len(a)):
    if a[i] not in D:
        E.append(int(a[i]))
    D.append(a[i])
#print(E)

def line(tab):
    for j in range(len(tab) - 1):
        if not isnan(tab[0, j]):
            return False
        if not isnan(tab[L - 1, j]) :
            return False
    return True


def linked(tab):
    if line(tab):
        return "No existing path between start and end"
    else:
        for k in range(len(a)):
            for i in range(int(binom(L - 2, int(a[k])))):
                A = np.zeros(L)
                B = np.zeros(L)
                sample = list(it.combinations(tab[1:-1], int(a[k])))[i]
                sample1 = sample + (tab[1],)
                sample2 = sample + (tab[L - 1],)
                for j in range(int(a[k] + 1)):
                    for t in range(int(a[k] + 1)):
                        # print(sample[j][t])
                        if not isnan(sample1[j][t]):
                            A[int(sample1[j][t] - 1)] = 1
                        if not isnan(sample2[j][t]):
                            B[int(sample2[j][t] - 1)] = 1
                # print(A)
                if (np.sum(A) == a[k] + 1 or np.sum(B) == a[k] + 1) and a[k] + 1 != L:
                    return "No existing path between start and end"
    return "There exists a path between start and end"

print(linked(tab))


def edgelist(file1, file2):

    F = open(file1, "r")
    f1 = F.read()
    f = f1.splitlines()
    F.close()
    for i in range(len(f)):
        f[i] = f[i].translate({ord(r): None for r in '[]'})
    g = open(file2, "w")

    for i in range(len(f)):
        tab = np.zeros(len(f[i].split()))
        for k in range(len(f[i].split())):
            tab[k] = float(f[len(f) - (i + 1)].split()[k])
        for j in range(len(f) - (i + 1)):
            J = len(f) - i - (j + 1)
            #print(len(f) - i, J, tab, J in tab)
            if J in tab:
                print(len(f) - i, J, distances[len(f) - i - 1, J - 1], file=g)
    g.close()
    return "Done"


print(edgelist("AllLinks", "edgesII"))

f = open("heuristicsII", "w")
for i in range(L):
    print(distances[i, L - 1], file=f)
f.close()






