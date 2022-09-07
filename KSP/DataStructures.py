import collections
import heapq

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
    
    def put(self, e):
        heapq.heappush(self.elements, e)
    
    def get(self):
        return heapq.heappop(self.elements)

    def peek(self, i=0):
        return self.elements[i] if len(self.elements) > 0 else (None, None)

    def remove(self, e):
        self.elements.remove(e)

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
        self.edges[u][v] = PathEdge(n, u, v, c, type)

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
