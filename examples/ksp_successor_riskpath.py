# Copyright 2023 Norwegian University of Science and Technology (NTNU)
# Author: Renan Guedes Maidana

#!/usr/bin/env python3

from KSP import KStar
from KSP.DataStructures import Edge, WeightedGraph

# Find paths s-t from sequences of sidetrack edges
def getPaths(ks, R):
    paths = []
    # The best path found by A* (seq == []) is found first
    path = [ks.t]; n = ks.t
    cost = 0
    while not n == ks.s:
        cost += ks.AStar.G.weight(ks.AStar.searchTree.T[n], n)
        n = ks.AStar.searchTree.T[n]
        path.append(n)
    paths.append([path, cost-1])
    R.pop(0)

    # Find the other paths
    for seq in R:
        n = ks.t; cost = 0
        path = [n]
        while not n == ks.s:
            if len(seq) > 0:
                sidetrack = seq[-1].split(':')[1]
                u, v = sidetrack.split(',')
                if n == v:
                    cost += ks.AStar.G.weight(u, n)
                    n = u
                    path.append(u)
                    seq.pop(-1)
                else:
                    cost += ks.AStar.G.weight(ks.AStar.searchTree.T[n], n)
                    n = ks.AStar.searchTree.T[n]
                    path.append(n)
            else:
                cost += ks.AStar.G.weight(ks.AStar.searchTree.T[n], n)
                n = ks.AStar.searchTree.T[n]
                path.append(n)
        paths.append([path, cost-1])
    return paths

simTree = {
    'S1':  {'prob': 0/100},
    'S2': {'prob': 10/100},
    'S3': {'prob':  5/100},
    'S4': {'prob': 20/100},
    'S5': {'prob': 15/100},
    'S6':  {'prob': 7/100},
    'S7': {'prob': 25/100},
    'S8': {'prob': 30/100},
    'S9': {'prob': 20/100},
    'S10': {'prob': 8/100},
    'end': {'prob': 1}
}

G = {
    'S1': ['S2', 'S3'],
    'S2': ['S4', 'S5'],
    'S3': ['S5', 'S6'],
    'S4': ['S7', 'S8'],
    'S5': ['S8', 'S9'],
    'S6': ['S9', 'S10'],
    'S7': ['end'],
    'S8': ['end'],
    'S9': ['end'],
    'S10':['end']  
}

from math import inf

class Node:
    def __init__(self, name, parent=None, data=None):
        self.name = name
        self.parent = parent
        self.data = data

if __name__ == "__main__":
    # Define the graph search algorithm
    # alg = Dijkstra()
    # alg = AStar(h=get_h)
    ks = KStar()

    def getW(u):
        uu = ks.AStar.searchTree.T[u]
        wu = 1
        while not uu == None:
            pc = 0.5*(simTree[uu]['prob']+simTree[u]['prob'])
            wu = (1-pc)*getW(uu)
            u = uu
            uu = ks.AStar.searchTree.T[u]
        return wu

    def successor(u):
        newEdges = []
        neighbors = G[u]
        if 'end' in neighbors:
            return [Edge(u=u,v='end',c=1)]
        for v in neighbors:
            pu = simTree[u]['prob']; pv = simTree[v]['prob']
            pc = 0.5*(pu+pv)
            pp = pc*getW(u)
            newEdges.append(Edge(u=u, v=v, c=pp))
        return newEdges

    # Set successor function for A*
    ks.AStar.successors = successor

    # Functions
    ks.AStar.cost = lambda u, v: ks.AStar.searchTree.g[u] + ks.AStar.G.weight(u, v)
    
    # Get the shortest path tree
    s, t = 'S1', 'end'      # Start and goal depend on the graph notation
    sequences = ks.search(s, t, k=inf)
    paths = getPaths(ks, sequences)
    
    # Print paths
    print()
    for i, p in enumerate(paths):
        path, c = p
        path.reverse()
        print('Path %d\t- cost %.4f: ' % (i+1, c), end='')
        for i in range(len(path)-1):
            print('%s->' % path[i], end='')
        print('%s' % path[-1])