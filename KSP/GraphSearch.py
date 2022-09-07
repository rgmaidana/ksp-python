from KSP.DataStructures import PriorityQueue, SearchTree, WeightedGraph

class Dijkstra:
    def __init__(self):
        self.queue = PriorityQueue()
        self.closed = []
        self.G = WeightedGraph()
        self.searchTree = SearchTree()
        # self.T = dict()     # Search tree
        # self.g = dict()     # Costs in the search tree

    def search(self, s, t):        
        # Start with s in the SPT, with cost 0
        self.queue.put(0, s)
        self.searchTree.T[s] = None
        self.searchTree.g[s] = 0

        # Search until there is nothing left to expand
        while not self.queue.empty():
            # Get the minimal cost node in the open queue
            u = self.queue.get()[1]

            # Goal found
            if u == t:
                break

            # Look at neighboors v of u and select some for expansion
            for v in self.G.neighboors(u):
                new_g = self.searchTree.g[u] + self.G.weight(u, v)
                if (not v in self.searchTree.T) or (new_g < self.searchTree.g[v]):
                    self.searchTree.g[v] = new_g
                    f = new_g
                    self.queue.put(f, v)
                    self.searchTree.T[v] = u
                
        return self.searchTree.T

# Inherits Dijkstra for the data structures
class AStar(Dijkstra):
    def __init__(self, h=None):
        super().__init__()
        self.h = h      # Handle for the new heuristic function

    def search(self, s, t):        
        # Start with s in the SPT, with cost 0
        self.queue.put(0, s)
        self.searchTree.T[s] = None
        self.searchTree.g[s] = 0

        # Search until there is nothing left to expand
        while not self.queue.empty():
            # Get the minimal cost node in the open queue
            u = self.queue.get()[1]
            
            # Goal found
            if u == t:
                break

            # Look at neighboors v of u and select some for expansion
            for v in self.G.neighboors(u):
                new_g = self.searchTree.g[u] + self.G.weight(u, v)
                if (not v in self.searchTree.T) or (new_g < self.searchTree.g[v]):
                    self.searchTree.g[v] = new_g
                    f = new_g + self.h(v, t)
                    self.queue.put(f, v)
                    self.searchTree.T[v] = u
                
        return self.searchTree.T
