import numpy as np
from math import atan2
from collections import OrderedDict
import csv


def Nearest(graph, node_key, node, no_links):
    a = [2]
    b = ["-1"]
    for key in graph.keys():
        graphnode = graph[key]
        if node_key not in no_links[key]:
            a.append((node[0] - graphnode[0]) ** 2 + (node[1] - graphnode[1]) ** 2)
            b.append(key)
    ind = np.argmin(a)
    key = b[ind]
    try:
        return key, graph[key]
    except:
        return "-1", [0, 0]

def distance(node1, node2):
    return np.sqrt((node1[0] - node1[0])**2 + (node1[1] - node2[1])**2)

def findNeighbors(graph, node_key, node, rad, obstacles, nearest_node, goal, d):
    inRad = []
    distances_sqr = []
    distances_sqr_goal = []
    copy = graph.copy()
    del copy[node_key]
    del copy[nearest_node]
    keys = list(copy.keys())
    for key in keys:
        neighbor = graph[key]
        distance_sqr = (node[0] - neighbor[0]) ** 2 + (node[1] - neighbor[1]) ** 2
        distance_sqr_goal = (neighbor[0] - goal[0]) ** 2 + (neighbor[1] - goal[1]) ** 2
        distances_sqr_goal.append(distance_sqr_goal)
        if not contact(node, neighbor, obstacles, d) and distance_sqr <= rad ** 2:
            #distances_sqr.append(distance_sqr)
            inRad.append(key)
    #best = inRad[np.argmin(distances_sqr_goal)]
    return inRad

def ChooseParent(graph, neighbor_nodes, nearest_node, new_node, obstacles, cost, delta):
    d = distance(graph[nearest_node], graph[new_node])
    c = cost[nearest_node] + d
    parent = nearest_node
    for node in neighbor_nodes:
        x1, x2 = graph[node], graph[new_node]
        dist = distance(x1, x2)
        #if not contact(x1, x2, obstacles, delta) and dist < d:
        #    parent = node
        #    d = dist
        if not contact(x1, x2, obstacles, delta) and cost[node] + dist < c:
            parent = node
            c = cost[node] + dist
    return parent

def real_x(x1, x2):
    if x1 >= 0 and x2 >= 0:
        return np.min([x1, x2])
    elif x1 >= 0 and x2 <= 0:
        return x1
    elif x1 <= 0 and x2 >= 0:
        return x2
    else:
        return None

def InCylinder(node, obstacles, delta):
    for i in range(len(obstacles)):
        x, y = obstacles[i, :2]
        r = obstacles[i, 2] + delta
        if (node[0] - x) ** 2 + (node[1] - y) ** 2 < r ** 2:
            return True
    return False

def contact(node1, node2, obstacles, d, limit=False):
    segment = np.array(node2) - np.array(node1)
    for i in range(len(obstacles)):
        x, y = obstacles[i, :2]
        r = obstacles[i, 2] + d/2
        a = np.sum(np.square(segment))
        x1, x2 = node1[0], node2[0]
        y1, y2 = node1[1], node2[1]
        b = 2 * (x1 * x2 - x * x1 - x2 ** 2 + x * x2) + 2 * (
                y1 * y2 - y * y1 - y2 ** 2 + y * y2)
        c = x ** 2 + y ** 2 + x2 ** 2 + y2 ** 2 - 2 * x * x2 - 2 * y * y2 - r ** 2
        delta = b ** 2 - 4 * a * c
        if delta > 0 and a != 0:
            x1, x2 = (-b - np.sqrt(delta)) / (2 * a), (-b + np.sqrt(delta)) / (2 * a)
            val = real_x(x1, x2)
            try:
                if ((not limit and val < 1) or (limit and val <= 1)) and val > 0:
                    return True
            except:
                pass
    return False

def Build_contact_node(node1, node2, obstacles, d, D):
    if inRing(node1, obstacles, d, D):
        # No building contact node if graphnode is in a ring
        return None
    segment = np.array(node2) - np.array(node1)
    k = 1
    node_xy = None
    for i in range(len(obstacles)):
        x, y = obstacles[i, :2]
        r = obstacles[i, 2] + D
        a = np.sum(np.square(segment))
        x1, x2 = node1[0], node2[0]
        y1, y2 = node1[1], node2[1]
        b = 2 * (x1 * x2 - x * x1 - x2 ** 2 + x * x2) + 2 * (
                    y1 * y2 - y * y1 - y2 ** 2 + y * y2)
        c = x ** 2 + y ** 2 + x2 ** 2 + y2 ** 2 - 2 * x * x2 - 2 * y * y2 - r ** 2
        delta = b ** 2 - 4 * a * c
        if delta > 0 and a != 0:
            X1, X2 = (-b - np.sqrt(delta)) / (2 * a), (-b + np.sqrt(delta)) / (2 * a)
            val = real_x(X1, X2)
            try:
                if val < k:
                    k = val
                    node_xy = [x1 * (1 - k) + x2 * k, y1 * (1 - k) + y2 * k]
            except:
                pass
        elif delta > 0 and a == 0:
            pass
    if node_xy == node1:
        return None
    else:
        # node1 can be in a ring "[r + d/2  -  r + d]" => node_xy might be None
        #assert np.prod([int((node_xy[0] - obstacles[i, 0]) ** 2 + (node_xy[1] - obstacles[i, 1]) ** 2 >= (obstacles[i, 2] + d) ** 2) for i in range(len(obstacles))]) == 1
        return node_xy


def inRing(node, obstacles, d, D):
    for i in range(len(obstacles)):
        x, y = obstacles[i, :2]
        r = obstacles[i, 2]
        if (r + d) ** 2 < (node[0] - x) ** 2 + (node[1] - y) ** 2 < (r + D) ** 2:
            return True
    return False


def RRTstar(allnodes, obstacles, n=100, rad=0.1, stepsize=0.001, delta=0.045, Delta=0.05):
    nodes = allnodes.copy()
    nodeStart = allnodes['0']
    graphNodes = {"0": nodeStart}
    N = len(nodes)
    Cost = {"0": 0}
    Parent = {key: None for key in allnodes.keys()}
    noLinks = {str(i): [] for i in range(len(allnodes))}
    unblocked = False
    i = 0
    remaining_free_nodes = allnodes.copy()
    del remaining_free_nodes["0"]
    goal = allnodes["goal"]
    test = False
    while Parent["goal"] is None and i < n-1:
        #print("iteration: ", i, Parent)
        if not list(remaining_free_nodes.keys()):
            print("Failure: Goal has not been reach")
            print(Parent)
            return False
        node = np.random.choice(list(remaining_free_nodes.keys()))
        nearest_node, nearest_xy = None, None
        nds = remaining_free_nodes.copy()
        #print(graphNodes)
        while not unblocked:
            nearest_node, nearest_xy = Nearest(graphNodes, node, nodes[node], noLinks)
            if nearest_node != "-1":
                unblocked = True
            else:
                del nds[node]
                node = np.random.choice(list(nds.keys()))
        unblocked = False
        #print(i, nearest_node)
        squared_norm = (nodes[node][0] - nearest_xy[0]) ** 2 + (nodes[node][1] - nearest_xy[1]) ** 2
        if squared_norm >= stepsize ** 2:
            new_node = str(N)
            test = True
            norm = np.sqrt(squared_norm)
            new_xy = [nearest_xy[0] + stepsize * (nodes[node][0] - nearest_xy[0]) / norm, nearest_xy[1] +
                      stepsize * (nodes[node][1] - nearest_xy[1]) / norm]
            nodes[new_node] = new_xy
            noLinks[new_node] = []
            #assert new_node != nearest_node
        else:
            new_node = node
            new_xy = nodes[node]
            #assert new_node != nearest_node
        #Cost[new_node] = Cost[nearest_node] + distance(nodes[new_node], nearest_xy)      # worst
        if contact(nodes[nearest_node], nodes[new_node], obstacles, delta):
            new_xy = Build_contact_node(nodes[nearest_node], nodes[new_node], obstacles, delta, Delta)
            noLinks[nearest_node].append(node)
            new_node = str(N)
            test = True
            #assert new_node != nearest_node
            if new_xy is not None:
                if InCylinder(new_xy, obstacles, Delta):  # elif ?
                    new_xy = None
                else:
                    nodes[new_node] = new_xy  # new node in then added to the graph
                    noLinks[new_node] = [node]
            else:
                try:
                    del nodes[new_node]
                except:
                    pass
        if test:
            N += 1
            test = False
        if new_xy is not None:
            graphNodes[new_node] = new_xy                                        # <-------------
            if new_node == node:
                noLinks[node] = []
            near_nodes = findNeighbors(graphNodes, new_node, new_xy, rad, obstacles, nearest_node, goal, delta)
            parent = ChooseParent(graphNodes, near_nodes, nearest_node, new_node, obstacles, Cost, delta)
            Parent[new_node] = parent
            Cost[new_node] = Cost[parent] + distance(graphNodes[new_node], graphNodes[parent])
            try:
                del remaining_free_nodes[new_node]
            except:
                pass
            #print(Parent)
        i += 1
    if Parent["goal"] is None:
        print("Failure !")
        return False
    else:
        print("SUCCESS !")
        value = "goal"
        inverse_path = {"goal": nodes["goal"]}
        while value != "0":
            value = Parent[value]
            inverse_path[value] = nodes[value]
        path = OrderedDict(reversed(list(inverse_path.items())))
        print(path)
        with open('nodes.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerows([path[key] for key in path.keys()])
        return True


if __name__ == "__main__":

    F = open("cylinders", "r")
    ffile = F.read()
    elts = ffile.splitlines()
    F.close()

    obstacles = np.empty([len(elts), 3])
    for j in range(len(obstacles)):
        x = float(elts[j].split()[0])
        y = float(elts[j].split()[1])
        r = float(elts[j].split()[2]) / 2
        obstacles[j, :] = [x, y, r]
    #print(obstacles)

    G = open("ScatteredNodes", "r")
    gfile = G.read()
    elts = gfile.splitlines()
    G.close()
    nodes = {}
    for j in range(len(elts) - 1):
        x = float(elts[j].split()[0])
        y = float(elts[j].split()[1])
        nodes[str(j)] = [x, y]
    nodes["goal"] = [float(elts[-1].split()[0]), float(elts[-1].split()[1])]
    #print(nodes)

    RRTstar(nodes, obstacles, n=5000, rad=0.1, stepsize=0.05)
