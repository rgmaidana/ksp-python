#!/usr/bin/env python3

from KSP import KStar
from KSP.DataStructures import Edge, WeightedGraph

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

def toygraph2():
    graph = WeightedGraph()

    # Add graph edges
    # Incoming s
    # Incoming 1
    graph.addEdge('0', '1', 1)
    # Incoming 2
    graph.addEdge('0', '2', 3)
    # Incoming 3
    graph.addEdge('1', '3', 3); graph.addEdge('2', '3', 1)
    # Incoming 4
    graph.addEdge('1', '4', 6); graph.addEdge('2', '4', 3)
    # Incoming t
    graph.addEdge('3', '5', 6); graph.addEdge('4', '5', 1)
    
    heuristic = {'0': 0, '1': 0, '2': 0, '3': 0, '4': 0, '5': 0}

    return graph, heuristic

def get_h(u, v):
    # heuristic = {'S0': 6, 'S1': 2, 'S2': 1, 'S3': 0}
    _, heuristic = kstar_ex_3_graph()
    # _, heuristic = kstar_ex_4_6_graph()
    # _, heuristic = toygraph2()
    return heuristic[u]

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
    paths.append([path, cost])
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
        paths.append([path, cost])
    return paths

if __name__ == "__main__":
    # Define the graph search algorithm
    # alg = Dijkstra()
    # alg = AStar(h=get_h)
    ks = KStar()

    # Initialize search algorithm with problem graph G
    graph, _ = kstar_ex_3_graph()
    # graph, _ = kstar_ex_4_6_graph()
    # graph, _ = toygraph2()

    # Successor function for A*
    def successor(u):
        neighbors = graph.neighbors(u)
        successors = []
        for v in neighbors:
            c = graph.edges[u][v].c
            newEdge = Edge(u=u,v=v,c=c)
            successors.append(newEdge)
        return successors

    def successor2(u):
        neighbors = graph.neighbors(u)
        successors = []
        for v in neighbors:
            c = graph.edges[u][v].c
            newEdge = Edge(u=u,v=v,c=c)
            successors.append(newEdge)
        return successors

    # Set successor function for A*
    ks.AStar.successors = successor

    # Set heuristic function handle for A*
    ks.AStar.h = get_h
    
    # Get the shortest path tree
    s, t = 'S0', 'S4'      # Start and goal depend on the graph notation
    sequences = ks.search(s, t, k=13)
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