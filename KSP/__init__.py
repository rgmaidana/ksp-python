# Copyright 2023 Norwegian University of Science and Technology (NTNU)
# Author: Renan Guedes Maidana
import copy
from typing import Type
from KSP.DataStructures import Edge, Node, PathGraph
from KSP.GraphSearch import AStar, Dijkstra

inf = float("inf")

# Modified Dijkstra for K*
class KS_Dijkstra(Dijkstra):
    def __init__(self):
        super().__init__()
        self.status = None
        self.virgin = True
        self.s = None
        self.t = None
        self.paths = {"*": Node("*")}

    # Indicates if a node should be expanded or not (e.g., according to branch-and-bound)
    # The default implementation returns "true"
    def shouldExpand(self, u):
        return True

    # Returns the path sigma in PG via which n was reached
    def sigma(self, n):
        sigma = [n]
        while not n == "*":
            n = self.searchTree.T[n]
            sigma.append(n)
        sigma.append("*")
        return sigma

# Modified K* AStar
class KS_AStar(AStar):
    def __init__(self):
        super().__init__()
        self.PG = PathGraph()
        self.tFound = False
        self.s, self.t = None, None
        self.current = None
        self.numedges = 0
        
        # Function handles
        self.successors = lambda _: []
        self.cost = lambda u, v: self.searchTree.g[u] + self.G.weight(u, v)
        self.h = lambda u, v: 0
        self.f = lambda u, v: self.cost(u, v) + self.h(v, self.t)

    def detourCost(self, u, v):
        return self.searchTree.g[u] + self.G.weight(u, v) - self.searchTree.g[v]

    # Adds incoming edge into Hin(v)
    def addIncoming(self, u, v):
        self.PG.addNode(v)
        newSidetrackEdge = Edge(u, v, self.detourCost(u, v))
        self.PG.nodes[v].inHeap.push(newSidetrackEdge, newSidetrackEdge.c)

    # Indicates if a node should be expanded or not (e.g., according to branch-and-bound pruning)
    # The default implementation returns "true"
    def shouldExpand(self, u):
        return True

    # Returns successors of u
    # For now, they are the neighbors of u in the graph G
    # Later, G will be implicit by the successor function
    def getOutgoingEdges(self, u):
        # Add successor edges to problem graph
        # NB! Successor function must return a list of (u,v,c) edges using the
        # DataStructure.Edges object
        for edge in self.successors(u):
            self.G.addEdge(edge.u, edge.v, edge.c)
            if not edge == self.t: self.numedges += 1
        return list(self.G.neighbors(u))

    # One expansion step for A*
    def doOneIteration(self):
        # Get node with minimal distance from the source
        _, self.current = self.queue.pop()

        # If target is chosen for expansion, set flag and exit
        if self.current == self.t:
            self.tFound = True
            return

        if self.shouldExpand(self.current):
            successors = self.getOutgoingEdges(self.current)
            for v in successors:
                new_g = self.cost(self.current, v)
                f = self.f(self.current, v)
                # v is in the SPT
                if v in self.searchTree.T:
                    # If the path via the edge (u,v) is better than the current SPT path (ud,v),
                    # then the old SPT path (ud,v) becomes a sidetrack edge
                    if new_g < self.searchTree.g[v]:
                        ud = self.searchTree.T[v]; fd = self.f(ud, v)
                        self.queue.remove((fd, v))
                        self.queue.push(v, f)
                        self.searchTree.g[v] = new_g
                        self.addIncoming(ud, v)
                        self.searchTree.T[v] = self.current
                    # If the path via the edge (u,v) is not better, then (u,v) is a sidetrack edge
                    else:
                        # if not v in self.closed:
                        self.addIncoming(self.current, v)
                # v is a new vertex
                else:
                    self.PG.addNode(v)
                    self.queue.push(v, f)
                    self.searchTree.g[v] = new_g
                    self.searchTree.T[v] = self.current
    
    def buildTreeHeaps(self, s):
        for v in self.searchTree.T:
            # If v is the start node, then Ht(s) is an empty heap
            if v == s:
                # rootIn(s) is added to Ht(s) if Hin(s) is not empty
                if not self.PG.nodes[s].inHeap.empty():
                    self.PG.nodes[s].THeap.push(self.PG.nodes[s].rootIn()[1], self.PG.nodes[s].rootIn()[0])
            else:
                # Let u be the predecessor of v in the SPT
                u = self.searchTree.T[v]
                # Ht(v) is constructed as a copy of Ht(u)
                self.PG.nodes[v].THeap = copy.deepcopy(self.PG.nodes[u].THeap)
                # Add rootIn(v) to Ht(v)
                # If Hin(v) is empty, then Ht(v) = Ht(u)
                for priority, item in self.PG.nodes[v].inHeap.elements:
                    self.PG.nodes[v].THeap.push(item, priority)

    def buildPathEdges(self):
        newPathEdges = []
        for pNode in self.PG.nodes:
            if not self.PG.nodes[pNode].THeap.empty():
                # Add cross edge to the root of pNode 
                nc, n = self.PG.nodes[pNode].rootT()
                n_name = '%s:%s,%s' % (pNode, n.u, n.v)
                if not self.PG.nodes[n.u].rootT()[1] == None:
                    Ru_c, Ru = self.PG.nodes[n.u].rootT()
                    Ru_name = '%s:%s,%s' % (Ru.v, Ru.u, Ru.v)
                    newPathEdges.append(self.PG.addEdge(pNode, n_name, Ru_name, c=Ru_c, type="cross"))
                for i in range(1,len(self.PG.nodes[pNode].THeap.elements)):
                    # There is a heap edge between pairs of nodes in THeap
                    # and a cross edge for each node
                    ni_c, ni = self.PG.nodes[pNode].THeap.elements[i]
                    # Cross edge of ni
                    ni_name = '%s:%s,%s' % (pNode, ni.u, ni.v)
                    if not self.PG.nodes[ni.u].rootT()[1] == None:
                        Ru_c, Ru = self.PG.nodes[ni.u].rootT()
                        Ru_name = '%s:%s,%s' % (Ru.v, Ru.u, Ru.v)
                        newPathEdges.append(self.PG.addEdge(pNode, ni_name, Ru_name, c=Ru_c, type="cross"))
                    # Heap edge between n and ni
                    newPathEdges.append(self.PG.addEdge(pNode, n_name, ni_name, c=ni_c-nc, type="heap"))
        return newPathEdges

    def buildPathGraph(self):
        self.buildTreeHeaps(self.s)
        return self.buildPathEdges()
    
    def search(self, lim):
        i = 0
        while True:
            # Search successors of A*
            self.doOneIteration(); i += 1
            # Decide if we continue with A*
            if self.tFound:
                return True
            if  i >= lim or self.queue.empty():
                return False

class KStar:
    def __init__(self, k=1):
        self.dijkstra = KS_Dijkstra()
        self.AStar = KS_AStar()
        self.status = None
        self.k = k
        self.R = []

        self.expansionLimit = inf
        
        self.s, self.t = None, None
        self.numIter = 0

        self.scheduling_F = lambda u: self.AStar.searchTree.g[u] + self.AStar.h(u, self.t)

    # Returns maximum distance d between the successors of n in Dijkstra's SPT
    def maxDist(self, n):
        neighbors = self.dijkstra.G.neighbors(n.id)
        if len(neighbors) == 0:
            return 0
        return max([self.dijkstra.searchTree.g[n] + self.dijkstra.G.weight(n.id,nd) for nd in self.dijkstra.G.neighbors(n.id)])

    # From K-Star-Java workbench:
    # This function is called after A* extended the path graph. It 
    # tries to bring Dijkstra's search in a consistent status. It 
    # establishes the path graph and explores those nodes, which 
    # are added into the path graph after their parent nodes have 
    # been expanded. 
    def tryToMaintainDijkstraSearch(self, newPathEdges):
        # First time running Dijkstra, so it is maintained by default
        if self.dijkstra.virgin:
            # Set PG as dijkstra's graph
            self.dijkstra.G = self.AStar.PG
            # If target node was found by AStar
            if self.AStar.tFound:
                # "*" is the first node of Dijkstra's graph
                R_node = Node('*')
                # Put R on Dijkstra's open queue
                self.dijkstra.queue.push(R_node, 0)
                self.dijkstra.searchTree.g[R_node] = 0
                
                # Get root of Ht(t)
                cost, root_T = self.dijkstra.G.nodes[self.t].rootT()
                # If Ht(t) is empty, then there are no sidetrack edges leading to the
                # target. Then we add "*" as the root of Ht(t)???
                if not root_T == None:
                    root_T_node = '%s:%s,%s' % (self.t, root_T.u, root_T.v)
                    self.dijkstra.G.addEdge(self.t, '*', root_T_node, c=cost, type="cross")
            self.dijkstra.virgin = False
            return True
        elif len(newPathEdges) > 0:
            return True
            # TODO Maintain dijkstra in case the heuristic is not admissible
            for edge in newPathEdges:
                u = edge.u; v = edge.v
                # New path edge
                # Add cross edge to the root of n and ni for u
                n, ni = u.split(':')[1].split(',')
                Rn_c, Rn = self.AStar.PG.nodes[n].rootT()
                if not Rn == None:
                    Rn_name = '%s:%s,%s' % (Rn.v, Rn.u, Rn.v)
                    self.dijkstra.G.addEdge(u, n, Rn_name, c=Rn_c, type="cross")
                Rni_c, Rni = self.AStar.PG.nodes[ni].rootT()
                if not Rni == None:
                    Rni_name = '%s:%s,%s' % (Rni.v, Rni.u, Rni.v)
                    self.dijkstra.G.addEdge(u, ni, Rni_name, c=Rni_c, type="cross")
                try:
                    # Add heap edge between n and ni
                    self.dijkstra.G.addEdge(u, n, ni, c=Rni_c-Rn_c, type="heap")
                except TypeError:
                    pass
                # Add cross edge to the root of n and ni for v
                n, ni = v.split(':')[1].split(',')
                Rn_c, Rn = self.AStar.PG.nodes[n].rootT()
                if not Rn == None:
                    Rn_name = '%s:%s,%s' % (Rn.v, Rn.u, Rn.v)
                    self.dijkstra.G.addEdge(v, n, Rn_name, c=Rn_c, type="cross")
                Rni_c, Rni = self.AStar.PG.nodes[ni].rootT()
                if not Rni == None:
                    Rni_name = '%s:%s,%s' % (Rni.v, Rni.u, Rni.v)
                    self.dijkstra.G.addEdge(v, ni, Rni_name, c=Rni_c, type="cross")
                try:
                    # Add heap edge between n and ni
                    self.dijkstra.G.addEdge(v, n, ni, c=Rni_c-Rn_c, type="heap")
                except TypeError:
                    pass
            return True

    # Returns the path sigma in PG via which n was reached
    def sigma(self, n):
        sigma = [n.id]
        while not n.parent == None:
            n = n.parent
            sigma.append(n.id)
        return sigma

    # Returns the sequence of sidetrack edges for sigma
    def getSeq(self, n):
        seq = [n.id] if not n.id == "*" else []
        while not n.parent == None:
            ni_1, ni = n, n.parent
            edge = self.dijkstra.G.edges[ni.id][ni_1.id]
            if not ni.id == "*":
                if edge.type == 'cross':
                    seq.append(ni.id)
                # else:
                #     seq.append(ni_1.id)
            n = n.parent
        return seq

    def updateStartGoal(self, s, t):
        self.s = s
        self.t = t
        self.AStar.s = s
        self.AStar.t = t
        self.dijkstra.s = s
        self.dijkstra.t = t

    # Implement K* search algorithm here
    # From page 2141 of K* paper
    def search(self, s, t, k=1, start_g=0):
        self.updateStartGoal(s, t)
        self.k = k

        # Start A* queue with initial node
        self.AStar.queue.push(s, 0)
        self.AStar.searchTree.T[s] = None
        self.AStar.searchTree.g[s] = start_g
        self.AStar.PG.addNode(s)

        # Run A* until t is selected for expansion
        # If no path is found, return nothing
        if not self.AStar.search(self.expansionLimit):
            print("Target {} not found, no paths available.".format(t))
            return
        # Refresh P(G)
        newPathEdges = self.AStar.buildPathGraph()
        # Assign whatever P(G) was found by A* (partial or total)
        # to Dijkstra's graph 
        self.tryToMaintainDijkstraSearch(newPathEdges)

        # Main loop (lines 8-25)
        resumeAStar = True
        while (not self.AStar.queue.empty()) or (not self.dijkstra.queue.empty()):
            # Lines 9-16
            if not self.AStar.queue.empty():
                if not self.dijkstra.queue.empty():
                    # Let u be the head of the search queue of A* and n the head of open_D
                    _, u = self.AStar.queue.peek()
                    _, n = self.dijkstra.queue.peek()
                    d = self.maxDist(n)                    
                    # Scheduling mechanism
                    f_u = self.scheduling_F(u)
                    if self.AStar.searchTree.g[self.t] + d <= f_u:
                        resumeAStar = False
                if resumeAStar:
                    # Resume A* in order to explore a larger portion of G
                    self.AStar.search(self.expansionLimit)
                    # Refresh P(G)
                    newPathEdges = self.AStar.buildPathGraph()
                    # Here we should try to make Dijkstra's search results consistent between
                    # rounds of A* (algorithm line 15). TODO?
                    # The paper says that P(G) does not need to be reconstructed or restructured 
                    # if we use a consistent heuristic. So we assume later on a consistent heuristic 
                    # will be available and don't worry about this part right now?
                    self.tryToMaintainDijkstraSearch(newPathEdges)
                    continue

            # Line 17
            resumeAStar = True
            if self.dijkstra.queue.empty():
                continue

            # Lines 18-25
            # Remove from open_D and place on closed_D the node n with the minimal d-value
            _, n = self.dijkstra.queue.pop()
            self.dijkstra.closed.append(n.id)
            for nd in self.dijkstra.G.neighbors(n.id):
                d_n = self.dijkstra.searchTree.g[n] + self.dijkstra.G.weight(n.id,nd)
                # Attach to nd a parent link referring to n
                node = Node(nd, parent=n)
                # Insert nd into open_D
                self.dijkstra.queue.push(node, d_n)
                self.dijkstra.searchTree.g[node] = d_n

            # Let sigma be the path in P(G) via which n was reached
            seq = self.getSeq(n)
            # Add seq(sigma) at the end of R
            self.R.append(seq)
            if len(self.R) == self.k:
                # Go to line 26
                break
            
        # Line 26
        return self.R