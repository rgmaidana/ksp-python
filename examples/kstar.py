#!/usr/bin/env python3

from KSP import KStar
from KSP.DataStructures import WeightedGraph

def kstar_ex_4_6_graph():
    graph = WeightedGraph()

    # Add graph edges
    # Incoming S0
    graph.addEdge('S3', 'S0', 9); graph.addEdge('S2', 'S0', 2)
    # Incoming S1
    graph.addEdge('S0', 'S1', 3); graph.addEdge('S2', 'S1', 1); graph.addEdge('S4', 'S1', 1)
    # Incoming S2
    graph.addEdge('S0', 'S2', 5); graph.addEdge('S1', 'S2', 4); graph.addEdge('S4', 'S2', 1)
    # Incoming S3
    graph.addEdge('S2', 'S3', 16); graph.addEdge('S4', 'S3', 7)
    # Incoming S4
    graph.addEdge('S2', 'S4', 1)
    # Incoming S5
    graph.addEdge('S3', 'S5', 3)
    # Incoming S6
    graph.addEdge('S1', 'S6', 7); graph.addEdge('S4', 'S6', 1); graph.addEdge('S5', 'S6', 4)

    # Heuristic values from each node to t=S6
    heuristic = {'S0': 6, 'S1': 6, 'S2': 2, 'S3': 7, 'S4': 1, 'S5': 3, 'S6': 0}

    return graph, heuristic

def kstar_ex_3_graph():
    graph = WeightedGraph()

    # Add graph edges
    graph.addEdge('S0', 'S1', 3); graph.addEdge('S0', 'S2', 2)
    graph.addEdge('S1', 'S1', 2); graph.addEdge('S1', 'S2', 1); graph.addEdge('S1', 'S4', 1)
    graph.addEdge('S2', 'S3', 1); graph.addEdge('S2', 'S4', 3)
    graph.addEdge('S3', 'S2', 2)

    heuristic = {'S0': 6, 'S1': 2, 'S2': 1, 'S3': 0, 'S4': 0}

    return graph, heuristic

def kstar_ex_3_graph_float():
    graph = WeightedGraph()

    # Add graph edges
    graph.addEdge('S0', 'S1', 3.1); graph.addEdge('S0', 'S2', 2.05)
    graph.addEdge('S1', 'S1', 2.2); graph.addEdge('S1', 'S2', 1.00006); graph.addEdge('S1', 'S4', 1.3)
    graph.addEdge('S2', 'S3', 1.5); graph.addEdge('S2', 'S4', 3.9)
    graph.addEdge('S3', 'S2', 2.1)

    heuristic = {'S0': 6, 'S1': 2, 'S2': 1, 'S3': 0, 'S4': 0}

    return graph, heuristic

def toygraph():
    graph = WeightedGraph()

    # Add graph edges
    graph.addEdge('S0', 'S1', 1)
    graph.addEdge('S1', 'S2', 1)
    graph.addEdge('S2', 'S3', 1)

    return graph

def component_graph():
    graph = WeightedGraph()

    Af = 1-1e-6
    Ar = 1-1e-2
    Bf = 1-1e-3
    Br = 1-1e-1
    Cf = 1-1e-9
    Cr = 1-1e-5
    
    # Add graph edges
    graph.addEdge('1', '2', Af); graph.addEdge('1', '3', Bf); graph.addEdge('1', '4', Cf)
    graph.addEdge('2', '1', Ar); graph.addEdge('2', '5', Bf); graph.addEdge('2', 'E1', Cf)
    graph.addEdge('3', '1', Br); graph.addEdge('3', '6', Af); graph.addEdge('3', 'E4', Cf)
    graph.addEdge('4', '1', Cr); graph.addEdge('4', 'E5', Af); graph.addEdge('4', 'E6', Bf)
    graph.addEdge('5', '2', Br); graph.addEdge('5', '3', Ar); graph.addEdge('5', 'E2', Cf)
    graph.addEdge('6', '2', Br); graph.addEdge('6', '3', Ar); graph.addEdge('6', 'E3', Cf)
    graph.addEdge('E1', 't', 0); graph.addEdge('E2', 't', 0); graph.addEdge('E3', 't', 0) 
    graph.addEdge('E4', 't', 0); graph.addEdge('E5', 't', 0); graph.addEdge('E6', 't', 0)

    heuristic = {'1': 0, '2': 0, '3': 0, '4': 0, '5': 0, '6': 0, 'E1': 0, 'E2': 0, 'E3': 0, 'E4': 0, 'E5': 0,  'E6': 0, 't': 0}

    return graph, heuristic

def get_h(u, v):
    # heuristic = {'S0': 6, 'S1': 2, 'S2': 1, 'S3': 0}
    # _, heuristic = kstar_ex_3_graph()
    _, heuristic = component_graph()
    return heuristic[u]

# Find paths s-t from sequences of sidetrack edges
def getPaths(ks, R):
    paths = []
    for seq in R:
        n = ks.t; cost = 0
        path = [n]
        while not n == ks.s:
            if len(seq) > 0:
                sidetrack = seq[-1].split(':')[1]
                u, v = sidetrack.split(',')
                if n == v:
                    cost *= 1-ks.AStar.G.weight(u, n)
                    n = u
                    path.append(u)
                    seq.pop(-1)
                else:
                    cost *= 1-ks.AStar.G.weight(ks.AStar.searchTree.T[n], n)
                    n = ks.AStar.searchTree.T[n]
                    path.append(n)
            else:
                cost *= 1-ks.AStar.G.weight(ks.AStar.searchTree.T[n], n)
                n = ks.AStar.searchTree.T[n]
                path.append(n)
        paths.append([path, cost])
    return paths

if __name__ == "__main__":
    # Define the graph search algorithm
    # alg = Dijkstra()
    # alg = AStar(h=get_h)
    ks = KStar()

    # Initialize search algorithm with problem graph G
    # ks.AStar.G = kstar_ex_3_graph()
    # ks.AStar.G, _ = kstar_ex_3_graph()
    # ks.AStar.G = toygraph()
    ks.AStar.G, _ = component_graph()

    # Set heuristic function handle for A*
    ks.AStar.h = get_h
    ks.AStar.cost = lambda u, v: ks.AStar.searchTree.g[u] + ks.AStar.G.weight(u, v)
    
    # Get the shortest path tree
    s, t = '1', 't'      # Start and goal depend on the graph notation
    sequences = ks.search(s, t, k=6)
    paths = getPaths(ks, sequences)
    
    # Print paths
    print()
    for i, p in enumerate(paths):
        path, _ = p
        path.reverse()
        u = path[0]
        print('Path %d\t: %s' % (i+1, u), end='')
        c = 1
        for j in range(1,len(path)-1):
            v = path[j]
            print('->%s' % v, end='')
            c *= 1-ks.AStar.G.weight(u, v)
            u = v
        print(' | cost: %.2E' % c)