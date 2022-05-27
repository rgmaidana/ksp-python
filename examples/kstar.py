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

    heuristic = {'S0': 0, 'S1': 0, 'S2': 0, 'S3': 0, 'S4': 0}

    return graph, heuristic

def toygraph():
    graph = WeightedGraph()

    # Add graph edges
    graph.addEdge('S0', 'S1', 2); graph.addEdge('S0', 'S2', 1)
    graph.addEdge('S1', 'S2', 3); graph.addEdge('S1', 'S3', 2)
    graph.addEdge('S2', 'S3', 3)

    return graph

def get_h(u, v):
    # heuristic = {'S0': 6, 'S1': 2, 'S2': 1, 'S3': 0}
    _, heuristic = kstar_ex_3_graph()
    # _, heuristic = kstar_ex_4_6_graph()
    return heuristic[u]

if __name__ == "__main__":
    # Define the graph search algorithm
    # alg = Dijkstra()
    # alg = AStar(h=get_h)
    ks = KStar()

    # Initialize search algorithm with problem graph G
    ks.AStar.G, _ = kstar_ex_3_graph()

    # Set heuristic function handle for A*
    ks.AStar.h = get_h
    
    # Get the shortest path tree
    s, t = 'S0', 'S4'      # Start and goal depend on the graph notation
    paths = ks.search(s, t, k=10)
    
    # Print paths
    print()
    for i, path in enumerate(paths):
        cost = 0
        path.reverse()
        print('Path %d: ' % (i+1), end='')
        for i in range(len(path)-1):
            print('%s->' % path[i], end='')
        print('%s' % path[-1])