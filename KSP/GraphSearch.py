# Copyright 2023 Norwegian University of Science and Technology (NTNU)
# Author: Renan Guedes Maidana
from KSP.DataStructures import PriorityQueue, SearchTree, WeightedGraph

class Dijkstra:
    def __init__(self, step=1):
        self.queue = PriorityQueue()
        self.closed = []
        self.G = WeightedGraph()
        self.searchTree = SearchTree()
        self.step = step
        # self.T = dict()     # Search tree
        # self.g = dict()     # Costs in the search tree

    def search(self, s, t):        
        # Start with s in the SPT, with cost 0
        self.queue.push(s, 0)
        self.searchTree.T[s] = None
        self.searchTree.g[s] = 0

        # Search until there is nothing left to expand
        while not self.queue.empty():
            # Get the minimal cost node in the open queue
            u = self.queue.pop()[1]

            # Goal found
            if u == t:
                break

            # Look at neighboors v of u and select some for expansion
            for v in self.G.neighbors(u, self.step):
                new_g = self.searchTree.g[u] + self.G.weight(u, v)
                if (not v in self.searchTree.T) or (new_g < self.searchTree.g[v]):
                    self.searchTree.g[v] = new_g
                    f = new_g
                    self.queue.put(f, v)
                    self.searchTree.T[v] = u
                
        return self.searchTree.T
    
class AStar:
    def __init__(self, h=lambda u, v: 0, step=1):
        self.queue = PriorityQueue()
        self.closed = []
        self.G = WeightedGraph()
        self.searchTree = SearchTree()
        self.step = step
        self.h = h      # Handle for the new heuristic function

    def search(self, s, t):        
        # Start with s in the SPT, with cost 0
        self.queue.push(s, 0)
        self.searchTree.T[s] = None
        self.searchTree.g[s] = 0

        # Search until there is nothing left to expand
        i = 0
        while not self.queue.empty():
            # print("Expanding node %d of %d" % (i, self.G.map.size))
            i += 1
            
            # Get the minimal cost node in the open queue
            u = self.queue.pop()[1]
            self.closed.append(u)
            
            # Goal found
            if u == t:
                print("Goal found at node %d" % i)
                break

            # Look at neighboors v of u and select some for expansion
            for v in self.G.neighbors(u, self.step):
                new_g = self.searchTree.g[u] + self.G.weight(u, v)
                if not v in self.closed:
                    if (not v in self.searchTree.T) or (new_g < self.searchTree.g[v]):
                        self.searchTree.g[v] = new_g
                        f = new_g + self.h(v, t)
                        self.queue.push(v, f)
                        self.searchTree.T[v] = u
                
        return self.searchTree.T
