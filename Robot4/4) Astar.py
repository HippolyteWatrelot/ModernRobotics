import numpy as np
from math import nan, isnan
import csv
import pandas as pd

edges = "edgesPRM"
heuristics = "heuristics"
na = open(heuristics, "r")
nb = na.read()
nc = nb.splitlines()
na.close()
nbnodes = len(nc)
print(nbnodes)

"""The edging function: it simply describes the direct edge between each node if they exist by displaying
the distance between them. The non directed links are displayed as 'nan'. The returned element is a 2D squared array"""

def edging(F, nbnodes):

    g = open(F)
    f1 = g.read()
    f = f1.splitlines()
    g.close()
    EdgeTab = np.zeros([nbnodes + 1, nbnodes + 1])                                                                      # I chose to label the rows and columns so it is 13*13 instead of 12*12
    EdgeTab[0, 0] = nan
    for j in range(nbnodes):
        EdgeTab[0, j+1] = str(j+1)
        EdgeTab[j+1, 0] = str(j+1)
    for i in range(len(f)):
        a = int(f[i].split()[0])
        b = int(f[i].split()[1])
        c = f[i].split()[2]
        EdgeTab[a, b] = c
        EdgeTab[b, a] = c
    for k in range(nbnodes):
        for m in range(nbnodes - k):
            if EdgeTab[k + 1, k + m + 1] == 0:
                EdgeTab[k + 1, k + m + 1] = nan
                EdgeTab[k + m + 1, k + 1] = nan
    return EdgeTab
print(edging(edges, nbnodes))


"""Just updating the algorithm grid by adding the heuristics for each node"""

grid = np.zeros([4, nbnodes])

g = open(heuristics, "r")
F = g.read()
f = F.splitlines()
g.close()
for i in range(nbnodes):
    grid[0, i] = nan
    grid[1, i] = f[i]
grid[0, 0] = 0
EdgeTab = edging(edges, nbnodes)
grid[2] = grid[1] + grid[0]
print(grid)



"""The complete A* Search function, (done with my own style: I will describe it progressively)"""

def Astar(grid, EdgeTab, nbnodes):

    ValNode = np.zeros(nbnodes)                                                                                         #The current values of the nodes (like for 6(20) means value = 20 for node 6)
    ClosedNode = np.zeros(nbnodes)                                                                                      #The closed nodes: array of 12 elements displaying 1 for the closed nodes and 0 if not.
    OpenNode = np.zeros(nbnodes)                                                                                        #The open nodes: array of 12 elements displaying 1 for the open nodes and 0 if not.
    ExploredNode = 1                                                                                                    #The current Explored node
    for j in range(nbnodes):                                                                                            #OpenNodeRed = np.empty(0):2*n array where n is the number of open nodes, first row is for label, second is for est tot cost
        ValNode[j] = nan
        ClosedNode[j] = 0
        OpenNode[j] = 0
    ValNode[0] = grid[2][0]
    while np.sum(ClosedNode) != nbnodes - 1:                                                                            #There, we explore the current explored node and search for the new open nodes
        print("ExploredNode", ExploredNode)
        for i in range(nbnodes):
            test = EdgeTab[int(ExploredNode), i + 1]
            if not isnan(test):
                if ClosedNode[i] == 0 and i != ExploredNode - 1:
                    if i != int(ExploredNode) - 1:
                        if grid[0][i] > grid[0][int(ExploredNode) - 1] + EdgeTab[int(ExploredNode), i + 1] or isnan(grid[0][i]):
                            grid[0][i] = grid[0][int(ExploredNode) - 1] + EdgeTab[int(ExploredNode), i + 1]
                            grid[2][i] = grid[1][i] + grid[0][i]
                            grid[3][i] = int(ExploredNode)
                            ValNode[i] = grid[2][i]
                        if isnan(grid[2][nbnodes - 1]) or grid[2][nbnodes - 1] >= grid[0][i]:
                            OpenNode[i] = 1
                        else:
                            ClosedNode[i] = 1
        if int((np.sum(ClosedNode))) == nbnodes - 2:                                                                    #this command is simply used to end the program when the last explored node updated the nodes est tot costs
            ClosedNode[int(ExploredNode) - 1] = 1
            break
        n = 0
        Tab1 = []
        Tab2 = []
        for k in range(len(OpenNode) - 1):                                                                              #Creating an array which will describe the open leaves of the explored node and their est tot costs
            if int(OpenNode[k]) == 1:
                if k != ExploredNode - 1:
                    n += 1
                    Tab1.append(ValNode[k])
                    Tab2.append(k + 1)
        Tab3 = np.zeros([2, n])
        for i in range(n):
            Tab3[0, i] = Tab2[i]
            Tab3[1, i] = Tab1[i]
        ClosedNode[int(ExploredNode) - 1] = 1
        OpenNode[int(ExploredNode) - 1] = 0
        OpenNodeRed = Tab3
        print("ClosedNode", ClosedNode)
        print("OpenNodeRed", OpenNodeRed)
        print("OpenNode", OpenNode)
        am = np.argmin(OpenNodeRed[1], axis=0)
        ExploredNode = OpenNodeRed[0, am]
    return grid




"""This function simply describe the best path taking the last row of the final grid"""

def path(tab):
    n = len(tab[-1])
    alpha = []
    path = []
    alpha.append(n)
    while n != 1:
        n = int(tab[-1, n - 1])
        alpha.append(n)
    k = len(alpha)
    for i in range(k):
        path.append(alpha[k - 1 - i])
    return path

"""Simple conversion path -> coordinates in space"""

def destinations(path, file):
    F = open(file)
    f1 = F.read()
    f = f1.splitlines()
    Tab = []
    for i in range(len(path)):
        Tab.append(f[path[i] - 1].split()[0:])
    return Tab



A = Astar(grid, EdgeTab, nbnodes)
print("final grid", A)
print()
B = path(A)
print("path", B)
C = destinations(B, "ScatteredNodes")
print("destinations", C)
Cx = []
Cy = []
for i in range(len(B)):
    Cx.append(float(C[i][0]))
    Cy.append(float(C[i][1]))
path = pd.DataFrame(list(zip(*[B, Cx, Cy])))
path2 = path.T
with open('LabelledPathPRM.csv', 'w', newline='') as file1:
    writer1 = csv.writer(file1)
    writer1.writerow([B[i] for i in range(len(B))])
with open('PRMpath.csv', 'w', newline='') as file2:
    writer2 = csv.writer(file2)
    for i in range(len(B)):
        writer2.writerow([float(Cx[i]), float(Cy[i]), 0])