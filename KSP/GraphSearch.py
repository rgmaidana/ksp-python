from KSP.DataStructures import PriorityQueue, WeightedGraph

class Dijkstra:
    def __init__(self):
        self.open = PriorityQueue()
        self.closed = []
        self.G = WeightedGraph()
        self.T = dict()     # Search tree
        self.g = dict()     # Costs in the search tree

    def search(self, s, t):        
        # Start with s in the SPT, with cost 0
        self.open.put(0, s)
        self.T[s] = None
        self.g[s] = 0

        # Search until there is nothing left to expand
        while not self.open.empty():
            # Get the minimal cost node in the open queue
            u = self.open.get()[1]

            # Goal found
            if u == t:
                break

            # Look at neighboors v of u and select some for expansion
            for v in self.G.neighboors(u):
                new_g = self.g[u] + self.G.weight(u, v)
                if (not v in self.T) or (new_g < self.g[v]):
                    self.g[v] = new_g
                    f = new_g
                    self.open.put(f, v)
                    self.T[v] = u
                
        return self.T

# Inherits Dijkstra for the data structures
class AStar(Dijkstra):
    def __init__(self, h=None):
        super().__init__()
        self.h = h      # Handle for the new heuristic function

    def search(self, s, t):        
        # Start with s in the SPT, with cost 0
        self.open.put(0, s)
        self.T[s] = None
        self.g[s] = 0

        # Search until there is nothing left to expand
        while not self.open.empty():
            # Get the minimal cost node in the open queue
            u = self.open.get()[1]
            
            # Goal found
            if u == t:
                break

            # Look at neighboors v of u and select some for expansion
            for v in self.G.neighboors(u):
                new_g = self.g[u] + self.G.weight(u, v)
                if (not v in self.T) or (new_g < self.g[v]):
                    self.g[v] = new_g
                    f = new_g + self.h(v, t)
                    self.open.put(f, v)
                    self.T[v] = u
                
        return self.T
