import numpy as np
from math import sqrt
from sympy.solvers import solve
from sympy import Symbol
import csv
import pandas as pd


"""I've done the RRT method here"""



def planner(X1, X2, file="cylinders2"):

    """This is the planner function: in the case that an obstacle blocks the way from the nearest node to the sample node,
    it returns the distance between the nearest node and the new node which (this one will be the impact node on the
    encountered obstacle).
    X1 and X2 are the coordinates of the candidate node (nearest node from the random node) of the graph and the
    random node"""

    F = open(file, "r")
    ffile = F.read()
    f = ffile.splitlines()
    F.close()
    tabsol = []
    alpha = Symbol('alpha')
    x = 1 / (1 + abs(alpha))
    """Here some technique to keep variable x in [0, 1]"""
    X = x * float(X1[0]) + (1 - x) * float(X2[0])
    Y = x * float(X1[1]) + (1 - x) * float(X2[1])
    for n in range(len(f)):
        x1 = float(f[n].split()[0])
        y1 = float(f[n].split()[1])
        r1 = float(f[n].split()[2]) / 2
        sol = solve((X - x1) ** 2 + (Y - y1) ** 2 - r1 ** 2, x)
        if sol:
            if "I" not in str(sol[0]):
                tabsol.append(sol[0] * float(X1[0]) + (1 - sol[0]) * float(X1[0]))
    absc = float(X1[0]) * np.ones(len(tabsol))
    arg = np.argmin(np.abs(absc - tabsol))
    minsolX = tabsol[int(arg)]
    solX = minsolX
    y = (minsolX - float(X2[0])) / (float(X1[0]) - float(X2[0]))
    solY = y * float(X1[1]) + (1 - y) * float(X2[1])
    MinimumDistance = sqrt((solX - X1[0]) ** 2 + (solY - X1[1]) ** 2)
    """This 'Minimum Distance' corresponds to the distance separating the candidate node from the first obstacle in 
    the direction of the random node"""
    return MinimumDistance


"""This RRT function will be fully described in an attached word file"""

def RRT(TreeSizeMax, file1="ScatteredNodes", file2="cylinders2"):

    """
    - f corresponds to the nodes list with the coordinates for each of them.
    - NoLinks is an array of tuples (two nodes labels) which role is to inform what are the known unlinkable nodes duo:
      useful to not repeat the nearest node detection.
    - path is the found path to be returned
    - Tree is an array of arrays (these ones describing potential partial paths) describing the full tree of
      the algorithm
    - TreeNodes contains and adds the graph nodes during the processus
    - positions is an array of tuples describing the positions of all the scattered nodes from file1, this array will
      then add new collision nodes during the processus
    - TreeSize simply indicates the current size of the tree.
    """

    positions = []
    F = open(file1, "r")
    f1 = F.read()
    f = f1.splitlines()
    F.close()
    NoLinks = []
    path = []
    for i in range(len(f)):
        positions.append((float(f[i].split()[0]), float(f[i].split()[1])))
    print("positions", positions)
    Tree = [[1]]
    """Tree initialized with node 1 (start)"""
    TreeNodes = [1]
    """Nodes in the tree, here initialized by 1"""
    TreeSize = 1
    g = open(file2, "r")
    g1 = g.read()
    file = g1.splitlines()
    number = len(f)                                          #this variable will be used to add the collision nodes (it is the initialized label)
    Tab = np.arange(len(f) - 1) + 2                          #Initialized list of scattered nodes where we will randomly choose the node to "reach" (here: all nodes but the first one)
    while TreeSize <= TreeSizeMax:
        CandidateNodes = []                                  #The candidates tree nodes: same as tree nodes after removing some useless nodes
        A = []                                               #It is an array which will sort the the candidates tree nodes positions from their distance to the random node
        rd = np.random.choice(Tab)
        for i in range(len(TreeNodes)):
            CandidateNodes.append(TreeNodes[i])
        for i in range(len(NoLinks)):
            if NoLinks[i][0] == rd:
                CandidateNodes.remove(NoLinks[i][1])
        for i in range(len(CandidateNodes)):
            if CandidateNodes[i] != rd:
                Xrd = float(positions[rd - 1][0])
                Yrd = float(positions[rd - 1][1])
                Xi = float(positions[int(CandidateNodes[i]) - 1][0])
                Yi = float(positions[int(CandidateNodes[i]) - 1][1])
                A.append((CandidateNodes[i], sqrt((Xrd - Xi) ** 2 + (Yrd - Yi) ** 2)))
        dtype = [('s', int), ('distance', float)]
        Arr = np.array(A, dtype=dtype)
        Arr = np.sort(Arr, order='distance')
        NearestNode = int(Arr[0][0])
        """NearestNode is the candidate node of the tree (the nearest node from the random node"""
        print("NearestNode(", rd, ")=", NearestNode)
        alpha = Symbol('alpha', real=True)
        x = 1 / (1 + abs(alpha))
        """Here some technique to keep variable x in [0, 1]"""
        Xnearest = float(positions[int(NearestNode - 1)][0])
        Ynearest = float(positions[int(NearestNode - 1)][1])
        Xrandom = float(f[int(rd - 1)].split()[0])
        Yrandom = float(f[int(rd - 1)].split()[1])
        X = x * Xrandom + (1 - x) * Xnearest
        Y = x * Yrandom + (1 - x) * Ynearest
        obstacle = 0
        n = 0                                                                           #indicator of obstacle: this variable will be incremented as each obstacle does not interfere in the link
        while obstacle == 0 and n < len(file):
            x1 = float(file[n].split()[0])
            y1 = float(file[n].split()[1])
            r1 = float(file[n].split()[2]) / 2
            sol = solve((X - x1) ** 2 + (Y - y1) ** 2 - r1 ** 2, alpha)
            if sol:
                if "I" in str(sol[0]):                                                  #Detecting the complex solutions (We are searching for real solutions)
                    n += 1
            if not sol:
                n += 1
            else:
                if "I" not in str(sol[0]):
                    obstacle = 1
                    NoLinks.append((rd, NearestNode))
                    print("PAF !")
                    """Simple indication of a collision"""
                    number += 1
                    dist = sqrt((Xrandom - Xnearest) ** 2 + (Yrandom - Ynearest) ** 2)
                    Xnew = Xnearest + planner([Xnearest, Ynearest], [Xrandom, Yrandom]) * (Xrandom - Xnearest) / dist
                    Ynew = Ynearest + planner([Xnearest, Ynearest], [Xrandom, Yrandom]) * (Yrandom - Ynearest) / dist
                    """New coordinates of the added node on the graph when it has been created after a collision"""
                    TreeNodes.append(number)
                    positions.append((Xnew, Ynew))
                    for cell in range(len(Tree)):
                        T = []
                        T1 = []
                        for i in range(len(Tree[cell])):
                            T.append(Tree[cell][i])
                            T1.append(Tree[cell][i])
                        if NearestNode in Tree[cell]:
                            n = 0
                            while NearestNode != int(Tree[cell][n]):
                                n += 1
                            if len(Tree[cell]) >= int(n + 2):
                                del T[n + 1:]
                                T.insert(int(n + 1), int(number))
                                if T not in Tree:
                                    Tree.append(T)
                            else:
                                T1.insert(int(n + 1), int(number))
                                Tree[cell] = T1
                    TreeSize += 1
                    if (Xnew - float(f[-1].split()[0])) ** 2 + (Ynew - float(f[-1].split()[1])) ** 2 <= 0.04:
                        """When this created node is near enough from the goal"""
                        print("Final Tree", Tree)
                        for i in range(len(Tree)):
                            if int(Tree[i][-1]) == number:
                                path = Tree[i]
                        pos = []
                        for j in range(len(path)):
                            pos.append(positions[int(path[j] - 1)])
                        print("SUCCESS !", path, pos)
                        return path, pos
        if n == len(file):
            """Then we know that random node and nearest node are linkable. No obstacle encountered"""
            TreeNodes.append(rd)
            Tab = np.delete(Tab, np.where(Tab == int(rd)))
            for cell in range(len(Tree)):
                T = []
                T1 = []
                for i in range(len(Tree[cell])):
                    T.append(Tree[cell][i])
                    T1.append(Tree[cell][i])
                if NearestNode in Tree[cell]:
                    n = 0
                    while NearestNode != int(Tree[cell][n]):
                        n += 1
                    if len(Tree[cell]) >= int(n + 2):
                        del T[n + 1:]
                        T.insert(int(n + 1), int(rd))
                        if T not in Tree:
                            Tree.append(T)
                    else:
                        T1.insert(int(n + 1), int(rd))
                        Tree[cell] = T1
            TreeSize += 1
            if rd == len(f):
                print("Final Tree", Tree)
                for i in range(len(Tree)):
                    if int(Tree[i][-1]) == len(f):
                        path = Tree[i]
                pos = []                                                     #The different positions of the found path
                for j in range(len(path)):
                    pos.append(positions[int(path[j] - 1)])
                print("SUCCESS !", path, pos)
                return path, pos
    print("FAILURE !", Tree)
    return None


#print(RRT(30))

rrt = RRT(50)
path = rrt[0]
positions = rrt[1]

positionsX = []
positionsY = []
for i in range(len(path)):
    positionsX.append(float(positions[i][0]))
    positionsY.append(float(positions[i][1]))
with open('LabelledPathRRT.csv', 'w', newline='') as file1:
    writer1 = csv.writer(file1)
    writer1.writerow([path[i] for i in range(len(path))])
with open('RRTpath.csv', 'w', newline='') as file2:
    writer = csv.writer(file2)
    for i in range(len(path)):
        writer.writerow([float(positionsX[i]), float(positionsY[i]), 0])