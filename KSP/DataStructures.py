# Copyright 2023 Norwegian University of Science and Technology (NTNU)
# Author: Renan Guedes Maidana
import collections
import heapq
from math import sqrt

class Queue:
    def __init__(self):
        self.elements = collections.deque()
    
    def empty(self):
        return len(self.elements) == 0

    def put(self, x):
        self.elements.append(x)

    def get(self):
        return self.elements.popleft()

class PriorityQueue:
    def __init__(self):
        self.elements = []
        
    def empty(self):
        return len(self.elements) == 0
    
    def push(self, item, priority):
        entry = (priority, item)
        heapq.heappush(self.elements, entry)
    
    def pop(self):
        return heapq.heappop(self.elements)

    def peek(self):
        return self.elements[0]

    def remove(self, e):
        if e in self.elements:
            self.elements.remove(e)
        heapq.heapify(self.elements)

# In K-Star-Workbench, DefaultVertex?
class Node:
    def __init__(self, id, parent=None, data=None):
        self.id = id
        self.data = data
        self.parent = parent
    
    # Apparently python3 doesn't like to compare tuples whose first element is
    # not unique...
    def __lt__(self, other):
        return self
    def __le__(self, other):
        return self

# In K-Star-Workbench, DefaultDirectedEdge?
class Edge:
    def __init__(self, u=None, v=None, c=0):
        self.u = u
        self.v = v
        self.c = c
    
    def __lt__(self, other):
        return self
    def __le__(self, other):
        return self

class WeightedGraph:
    def __init__(self):
        self.nodes = dict()
        self.edges = dict()

    def addNode(self, n, data=None):
        if not n in self.nodes.keys():
            self.nodes[n] = Node(n, data=data)

    def addEdge(self, u, v, c=0):
        self.addNode(u)
        self.addNode(v)
        if not u in self.edges.keys():
            self.edges[u] = {}
        self.edges[u][v] = Edge(u=u, v=v, c=c)

    def neighbors(self, n):
        return list(self.edges[n].keys()) if n in self.edges else []

    def weight(self, u, v):
        return self.edges[u][v].c

class Grid:
    def __init__(self, origin, width, height, resolution=1):
        self.origin = origin
        self.width = width
        self.height = height
        self.resolution = resolution    # m/grid pos
        self.map = None
        self.weights = {}

    def meter2grid(self, x, y):
        grid_x = (x - self.origin[0]) / self.resolution
        grid_y = (y - self.origin[1]) / self.resolution
        return int(grid_x), int(grid_y)
    
    def grid2meter(self, grid_x, grid_y):
        x = (grid_x * self.resolution) + self.origin[0]
        y = (grid_y * self.resolution) + self.origin[1]
        return x, y
    
    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1)
    
    def weight(self, u, v):
        return 1
    
    def neighbors(self, id, step):
        (x, y) = id
        x_meter, y_meter = self.grid2meter(x, y)
        neighbors = [(x+step, y),   (x, y-step),
                     (x-step, y),   (x, y+step),
                     (x+step, y+step), (x+step, y-step),
                     (x-step, y+step), (x-step, y-step)]
        if (x+y) % 2 == 0:
            neighbors.reverse()
        results = []
        for neighbor in neighbors:
            grid_x, grid_y = neighbor
            x_neighbor, y_neighbor = self.grid2meter(grid_x, grid_y)
            dist = sqrt((x_meter-x_neighbor)**2 + (y_meter-y_neighbor)**2)
            if 0 <= grid_x < self.width and 0 <= grid_y < self.height and dist > 50:
                if self.map[grid_y, grid_x] == 0:
                    results.append(neighbor)
                else:
                    continue
        return results

class PathNode(Node):
    def __init__(self, id, data=None):
        super().__init__(id, data)
        self.inHeap = PriorityQueue()
        self.THeap = PriorityQueue()
    
    def rootIn(self):
        if not self.inHeap.empty():
            return self.inHeap.elements[0]
        return None, None

    def rootT(self):
        if not self.THeap.empty():
            return self.THeap.elements[0]
        return None, None

class PathEdge(Edge):
    def __init__(self, n, u, v, c, type=None):
        super().__init__(u, v, c)
        self.n = n
        self.type = type

class PathGraph(WeightedGraph):
    def __init__(self):
        super().__init__()
        self.isEstablished = False

    def addNode(self, n, data=None):
        if not n in self.nodes.keys():
            self.nodes[n] = PathNode(n, data=data)

    def addEdge(self, n, u, v, c=None, type=None):
        if not u in self.edges.keys():
            self.edges[u] = {}
        pEdge = PathEdge(n, u, v, c, type)
        self.edges[u][v] = pEdge
        return pEdge

class SearchTree:
    def __init__(self):
        self.opened = dict()
        self.closed = dict()
        self.T = dict()
        self.g = dict()
    
    def close(self, v):
        # Remove from open
        self.opened.pop(v)
        # Insert in closed
        self.closed[v] = 1

    def isClosed(self, v):
        return v in self.closed.keys()

    def open(self, v):
        if not self.isOpen(v):
            self.opened[v] = 1
    
    def isOpen(self, v):
        return v in self.opened.keys()
