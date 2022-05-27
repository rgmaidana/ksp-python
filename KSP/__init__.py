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
        self.paths = {'R': Node('R')}

    # Indicates if a node should be expanded or not (e.g., according to branch-and-bound)
    # The default implementation returns "true"
    def shouldExpand(self, u):
        return True

    # Returns successors of u
    # For now, they are the neighbors of u in the graph G
    # Later, G will be implicit by the successor function
    def getOutgoingEdges(self, u):
        return self.G.neighboors(u)

    # Returns the path sigma in PG via which n was reached
    def sigma(self, n):
        sigma = [n]
        while not n == 'R':
            n = self.T[n]
            sigma.append(n)
        sigma.append('R')
        return sigma

    # Single-step dijkstra search
    def doOneIteration(self):
        self.iterations += 1
        n = self.open.get()[1]
        if self.shouldExpand(n):
            successors = self.getOutgoingEdges(n)
            for v in successors:
                new_g = self.g[n] + self.G.weight(n, v)
                if (not v in self.T) or (new_g < self.g[v]):
                    self.g[v] = new_g
                    f = new_g
                    self.open.put((f, v))
                    self.T[v] = n
        return self.sigma(n)


# Modified K* AStar
class KS_AStar(AStar):
    def __init__(self, h=None):
        super().__init__(h)
        self.PG = PathGraph()
        self.tFound = False
        self.shouldContinue = True
        self.s, self.t = None, None

    def detourCost(self, u, v):
        return self.g[u] + self.G.weight(u, v) - self.g[v]

    def addIncoming(self, n, u, v):
        self.PG.addNode(v)
        newSidetrackEdge = Edge(u, v, self.detourCost(u, v))
        self.PG.nodes[v].inHeap.put((newSidetrackEdge.c, newSidetrackEdge))
    
    def buildTreeHeaps(self, s):
        for v in self.T:
            # If v is the start node, then Ht(s) is an empty heap
            if v == s:
                # rootIn(s) is added to Ht(s) if Hin(s) is not empty
                if not self.PG.nodes[s].inHeap.empty():
                    self.PG.nodes[s].THeap.put(self.PG.nodes[s].inHeap.elements[0])
            else:
                # Let u be the predecessor of v in the SPT
                u = self.T[v]
                # Ht(v) is constructed as a copy of Ht(u)
                self.PG.nodes[v].THeap = copy.deepcopy(self.PG.nodes[u].THeap)
                # Add rootIn(v) to Ht(v)
                # If Hin(v) is empty, then Ht(v) = Ht(u)
                for e in self.PG.nodes[v].inHeap.elements:
                    self.PG.nodes[v].THeap.put(e)

    def buildPathEdges(self):
        for pNode in self.PG.nodes:
            if not self.PG.nodes[pNode].THeap.empty():
                # Add cross edge to the root of pNode 
                nc, n = self.PG.nodes[pNode].rootT()
                n_name = '%s:%s,%s' % (pNode, n.u, n.v)
                Ru_c, Ru = self.PG.nodes[n.u].rootT()
                Ru_name = '%s:%s,%s' % (Ru.v, Ru.u, Ru.v)
                self.PG.addEdge(pNode, n_name, Ru_name, c=Ru_c, type="cross")
                for i in range(1,len(self.PG.nodes[pNode].THeap.elements)):
                    # There is a heap edge between pairs of nodes in THeap
                    # and a cross edge for each node
                    ni_c, ni = self.PG.nodes[pNode].THeap.elements[i]
                    # Cross edge of ni
                    ni_name = '%s:%s,%s' % (pNode, ni.u, ni.v)
                    Ru_c, Ru = self.PG.nodes[ni.u].rootT()
                    Ru_name = '%s:%s,%s' % (Ru.v, Ru.u, Ru.v)
                    self.PG.addEdge(pNode, ni_name, Ru_name, c=Ru_c, type="cross")
                    # Heap edge between n and ni
                    self.PG.addEdge(pNode, n_name, ni_name, c=ni_c-nc, type="heap")

    def buildPathGraph(self):
        self.buildTreeHeaps(self.s)
        self.buildPathEdges()

        # Debug
        # numEdges = 0
        # for e1 in self.PG.edges:
        #     for e2 in self.PG.edges[e1]:
        #         typ = self.PG.edges[e1][e2].type
        #         n = self.PG.edges[e1][e2].n
        #         # Special case for R node...
        #         if not self.PG.edges[e1][e2].u == 'R':
        #             u = self.PG.edges[e1][e2].u.split(':')[1]
        #         else:
        #             u = 'R'
        #         v = self.PG.edges[e1][e2].v.split(':')[1]
        #         c = self.PG.edges[e1][e2].c
        #         print("{} edge at path node {}:\t{} --> {}, cost: {}".format(typ, n, u, v, c))
        #         numEdges += 1
        # print("Number of path edges: %d" % numEdges)

    # Indicates if a node should be expanded or not (e.g., according to branch-and-bound pruning)
    # The default implementation returns "true"
    def shouldExpand(self, u):
        return True

    # Returns successors of u
    # For now, they are the neighbors of u in the graph G
    # Later, G will be implicit by the successor function
    def getOutgoingEdges(self, u):
        return self.G.neighboors(u)

    # One expansion step for A*
    def doOneIteration(self):
        # Get node with minimal distance from the source
        u = self.open.get()[1]
        # Put it in the closed list
        self.closed.append(u)

        # If target is chosen for expansion, set flag and exit
        if u == self.t:
            self.tFound = True
            return

        if self.shouldExpand(u):
            successors = self.getOutgoingEdges(u)
            for v in successors:
                new_g = self.g[u] + self.G.weight(u, v)
                # If v is not in the shortest path tree, it hasn't been visited before,
                # so the (u,v) edge is inserted in the SPT
                if (not v in self.T):
                    self.PG.addNode(v)
                    f = new_g + self.h(v, self.t)
                    self.open.put((f, v))
                    self.g[v] = new_g
                    self.T[v] = u             
                else:
                    # If v is in the SPT and the path via the edge (u,v) is better, then the old SPT path 
                    # becomes a sidetrack edge
                    if (new_g < self.g[v]):
                        f = new_g + self.h(v, self.t)
                        self.open.put((f, v))
                        self.g[v] = new_g
                        self.addIncoming(v, self.T[v], v)
                        self.T[v] = u    
                    # If v is in the SPT but the path via the edge (u,v) is not better, then that edge 
                    # is a sidetrack edge
                    else:
                        self.addIncoming(v, u, v)
    
    def search(self, lim):
        i = 0
        while self.shouldContinue:
            # Search successors of A*
            self.doOneIteration(); i += 1
            # Decide if we continue with A*
            if self.tFound or i >= lim or self.open.empty():
                self.shouldContinue = False
        self.shouldContinue = True


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

    # Returns maximum distance d between the successors of n in Dijkstra's SPT
    def maxDist(self, n):
        return max([self.dijkstra.g[n] + self.dijkstra.G.weight(n.id,nd) for nd in self.dijkstra.getOutgoingEdges(n.id)])

    # From K-Star-Java workbench:
    # This function is called after A* extended the path graph. It 
    # tries to bring Dijkstra's search in a consistent status. It 
    # establishes the path graph and explores those nodes, which 
    # are added into the path graph after their parent nodes have 
    # been expanded. 
    def tryToMaintainDijkstraSearch(self):
        # Set PG as dijkstra's graph
        self.dijkstra.G = self.AStar.PG
        # First time running Dijkstra, so it is maintained by default
        if self.dijkstra.virgin:
            # If target node was found by AStar
            if self.AStar.tFound:
                # "fancy R" is the first node of Dijkstra's graph
                cost, root_T = self.AStar.PG.nodes[self.t].rootT()
                root_T_node = '%s:%s,%s' % (self.t, root_T.u, root_T.v)
                R_node = Node('R')
                self.dijkstra.G.addEdge(self.t, 'R', root_T_node, c=cost, type="cross")
                # Put R on Dijkstra's open queue
                self.dijkstra.open.put((0, R_node))
                self.dijkstra.g[R_node] = 0
            self.dijkstra.virgin = False
            return True
        else:
            # TODO Maintain dijkstra in case the heuristic is not admissible
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
        seq = [n.id] if not n.id == 'R' else []
        while not n.parent == None:
            ni_1, ni = n, n.parent
            edge = self.dijkstra.G.edges[ni.id][ni_1.id]
            if not ni.id == 'R':
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
    def search(self, s, t, k=1):
        self.updateStartGoal(s, t)
        self.k = k

        # Start A* queue with initial node
        self.AStar.open.put((0, s))
        self.AStar.T[s] = None
        self.AStar.g[s] = 0
        self.AStar.PG.addNode(s)

        # Run A* until t is selected for expansion
        self.AStar.search(self.expansionLimit)
        # Refresh P(G)
        self.AStar.buildPathGraph()
        # Assign whatever P(G) was found by A* (partial or total)
        # to Dijkstra's graph 
        self.tryToMaintainDijkstraSearch()

        # Main loop (lines 8-25)
        while (not self.AStar.open.empty()) or (not self.dijkstra.open.empty()):
            # Lines 9-16
            if not self.AStar.open.empty():
                if not self.dijkstra.open.empty():
                    # Let u be the head of the search queue of A* and n the head of open_D
                    u, n = self.AStar.open.peek()[1], self.dijkstra.open.peek()[1]
                    d = self.maxDist(n)                    
                    # Scheduling mechanism
                    f_u = self.AStar.g[u] + self.AStar.h(u, self.t)
                    if self.AStar.g[self.t] + d <= f_u:
                        pass
                    else:
                        # Resume A* in order to explore a larger portion of G
                        self.AStar.halt = False
                        self.AStar.search(self.expansionLimit)
                        # Refresh P(G)
                        self.AStar.buildPathGraph()
                        # Here we should try to make Dijkstra's search results consistent between
                        # rounds of A* (algorithm line 15). TODO?
                        # The paper says that P(G) does not need to be reconstructed or restructured 
                        # if we use a consistent heuristic. So we assume later on a consistent heuristic 
                        # will be available and don't worry about this part right now.
                        self.tryToMaintainDijkstraSearch()
                        continue

            # Line 17
            if self.dijkstra.open.empty():
                continue

            # Lines 18-25
            # Remove from open_D and place on closed_D the node n with the minimal d-value
            n = self.dijkstra.open.get()[1]
            self.dijkstra.closed.append(n.id)
            for nd in self.dijkstra.getOutgoingEdges(n.id):
                d_n = self.dijkstra.g[n] + self.dijkstra.G.weight(n.id,nd)
                # Attach to nd a parent link referring to n
                node = Node(nd, parent=n)
                # Insert nd into open_D
                self.dijkstra.open.put((d_n, node))
                self.dijkstra.g[node] = d_n
            # Let sigma be the path in P(G) via which n was reached
            seq = self.getSeq(n)
            # Add seq(sigma) at the end of R
            self.R.append(seq)
            if len(self.R) == self.k:
                # Go to line 26
                break
        # Line 26
        return self.R