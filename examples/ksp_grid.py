# Copyright 2023 Norwegian University of Science and Technology (NTNU)
# Author: Renan Guedes Maidana

#!/usr/bin/env python3

from KSP import KStar
from KSP.DataStructures import Grid, Edge
from math import sqrt
import time

# Find paths s-t from sequences of sidetrack edges
def getPaths(ks, R):
    paths = []
    # The best path found by A* (seq == []) is found first
    n = str(ks.t).strip('()')
    path = [n]
    cost = 0
    while not n == str(ks.s).strip('()'):
        n1, n2 = n.split(','); nn = (int(n1), int(n2))
        cost += ks.AStar.G.weight(ks.AStar.searchTree.T[nn], nn)
        n = str(ks.AStar.searchTree.T[nn]).strip('()')
        path.append(n)
    paths.append([path, cost])
    R.pop(0)

    # Find the other paths
    for seq in R:
        n = str(ks.t).strip('()'); cost = 0
        path = [n]
        while not n == str(ks.s).strip('()'):
            n1, n2 = n.split(','); nn = (int(n1), int(n2))
            if len(seq) > 0:
                sidetrack = seq[-1].split(':')[1]
                u, v = sidetrack.split('),(')
                u = u.strip('('); v = v.strip(')')
                if n == v:
                    u1, u2 = u.split(','); uu = (int(u1), int(u2))
                    cost += ks.AStar.G.weight(uu, nn)
                    n = u
                    path.append(u)
                    seq.pop(-1)
                else:
                    cost += ks.AStar.G.weight(ks.AStar.searchTree.T[nn], nn)
                    n = str(ks.AStar.searchTree.T[nn]).strip('()')
                    path.append(n)
            else:
                cost += ks.AStar.G.weight(ks.AStar.searchTree.T[nn], nn)
                n = str(ks.AStar.searchTree.T[nn]).strip('()')
                path.append(n)
        paths.append([path, cost])
    return paths

if __name__ == "__main__":
    # Define the graph search algorithm
    # alg = Dijkstra()
    # alg = AStar(h=get_h)
    ks = KStar()

    # Initialize grid as problem graph for K*
    graph = Grid(origin=(0,0), width=10, height=10, resolution=1)
    
    numNodes = 1

    # Successor function for A*
    def successor(u):
        global numNodes
        neighbors = graph.neighbors(u)
        successors = []
        for v in neighbors:
            numNodes += 1
            newEdge = Edge(u=u,v=v,c=1)
            successors.append(newEdge)
        return successors

    # Set successor function for A*
    ks.AStar.successors = successor
    # ks.AStar.h = lambda v, t: sqrt((v[0]-t[0])**2 + (v[1]-t[1])**2)   # Euclidean distance

    # Get the shortest path tree
    s, t = graph.origin, (9,9)      # Start and goal depend on the graph notation
    t1 = time.time()
    K = 1000000
    sequences = ks.search(s, t, k=K)
    print('Time for %d paths: %.4f seconds' % (K, (time.time()-t1)))
    paths = getPaths(ks, sequences)
    print('Nodes explored: %d' % numNodes)
    
    # Plot paths
    import matplotlib.pyplot as plt
    plt.figure()
    legend = []
    for i, p in enumerate(paths):
        xl, yl = [], []
        path, c = p
        path.reverse()
        for loc in path:
            x, y = loc.split(',')
            xl.append(int(x)); yl.append(int(y))
        plt.plot(xl,yl)
        legend.append('Path %d (%d)' % (i+1, c))
    # plt.legend(legend)
    plt.axis([-0.5, 9.5, -0.5, 9.5])
    plt.show()