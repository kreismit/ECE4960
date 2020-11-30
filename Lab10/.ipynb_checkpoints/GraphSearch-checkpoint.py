#!/usr/bin/python
import numpy as np
import time

class search:
    def __init__(self, map):
        # Input: map (2-D numpy array, grid generated previously)
        self.map = np.stack([map,map,map,map],axis=-1) # map is the same for all angles
        self.maxRow = np.shape(self.map)[0]
        self.maxCol = np.shape(self.map)[1]
        self.cameFrom = np.zeros([self.maxRow, self.maxCol, 4, 3]).astype(int)
        # lookup for reached cells and direction
        self.cost = np.zeros([self.maxRow, self.maxCol, 4]).astype(int)
        # lookup matrix for cell costs
        self.solution = None # have we found a solution? if so, this is it

    def neighbors(self, node):
        # Find neighbors in a 3-D graph
        # Input: node (list or tuple of 3)
        # Output: out (list of tuples of 3)
        neighborList = []
        out = []
        #for node in nodes:
        #directions = [[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]]
        # Can't drive sideways.
        if node[2] == 0 or node[2] == 2:
            directions = [[1,0,0],[-1,0,0],[0,0,1],[0,0,-1]]
        elif node[2] == 1 or node[2] == 3:
            directions = [[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]]
        else:
            raise Exception("Out-of-bound rotation")
        neighbor = [0,0,0]
        for dir in directions: # loop through all possible directions
            for dim in range(3): # loop through 3 dimensions
                neighbor[dim] = node[dim]+dir[dim]
            # Normalize angles
            if neighbor[2] < 0:
                neighbor[2] = neighbor[2] + 4
            elif neighbor[2] > 3:
                neighbor[2] = neighbor[2] - 4
            # Make sure the output is not out-of-bounds
            if (neighbor[0] >= 0 and neighbor[1] >= 0
               and neighbor[0] < self.maxRow and neighbor[1] < self.maxCol
               and not (self.map[neighbor[0],neighbor[1],neighbor[2]])):
                # and doesn't coincide with a wall
                neighborList.append(tuple(neighbor))
        [out.append(x) for x in neighborList if x not in out] # no duplicates
        return out

    def frontier(self,nodes):
        # Determine which cell neighbors are the frontier
        # Input: nodes: list of nodes, where each node is a tuple: (x,y,th)
        # Output: out: list of "frontier" nodes, same format as input.
            out = []
            for node in nodes: # for each node given
                neighborList = self.neighbors(node)
                for i in neighborList: # for each neighbor (x,y,th),
                    if (self.cameFrom[i[0],i[1],i[2],:]==[0,0,0]).all():
                        # if the cameFrom matrix for it empty (never been there),
                        out.append(i) # it belongs in the frontier.
                        self.cameFrom[i[0],i[1],i[2],:] = node[:]
                        # and we came from this node
                        self.findCost(node,i) # fill in cost array
            return out

    def astar(self, start, goal):
        # Depth-first search (with cost)
        # Inputs: start: 1-D numpy array of 2 or 3 coordinates
        #         and goal: 1-D numpy array of 2 or 3 coordinates
        # Output: solution: list of nodes leading to goal (start to end)
        
        if len(start) is 2:         # if the input is just [x,y] or (x,y)
            start = list(start)     # make sure it's mutable
            start.append(0)         # add an angle coordinate
            start = tuple(start)    # and make it immutable to avoid problems
        if len(goal) is 2:          # if the input is just [x,y] or (x,y)
            goal = list(goal)       # make sure it's mutable
            goal.append(0)          # add an angle coordinate
            goal = tuple(goal)      # and make it immutable to avoid problems

        here = [start]              # We are at the starting point.
        startTime = time.time()
        #print("Starting at ({},{},{})\n".format(*here[0]))
        next = self.frontier(here)  # Take the first look around.
        # next is the queue of nodes to check next.
        #print("Trying these cells first: " + str(next) + "\n")
        bestPriority = 1000
        for i in next: # i is a set of coordinates in list next
            if i[:] == goal[:]: # we're done!
                print("Found a solution after {0:1.3f} seconds\n".format(time.time()-startTime))
                self.solution = self.genPath(start,goal)
            priority = 2*self.heuristic(i,goal) \
                     + self.cost[i[0],i[1],i[2]]
            if (priority < bestPriority):   # better than the best so far!
                bestPriority = priority     # this is the new best
                next.insert(0,next.pop(next.index(i)))  # move it to the front
        #print("After sorting: " + str(next) + "\n")
        
        #while not not(next) and self.solution is None:
        while not not(next):
        # keep searching until soln found OR the frontier is empty
            
            #print("Trying these cells next: " + str(here) + "\n")
            # Loosely sort in order of best heuristic and cost.
            # In the same loop, mark "came from" cells.
            bestPriority = 1000
            for i in next: # i is a set of coordinates in list next
                if i[:] == goal[:]: # we're done!
                    print("Found a solution after {0:1.3f} seconds\n".format(time.time()-startTime))
                    solution = self.genPath(start,goal)
                    solution.reverse()
                    return solution
                priority = 2*self.heuristic(i,goal) \
                         + self.cost[i[0],i[1],i[2]]
                if (priority < bestPriority):   # better than the best so far!
                    bestPriority = priority     # this is the new best
                    next.insert(0,next.pop(next.index(i)))  # move it to the front
            #print("After sorting: " + str(next) + "\n")
            here = next
            next = self.frontier(here) # generate next queue
                
    
    def heuristic(self, here, goal):
    # Just Manhattan distance.
    # here is the current pose (or x,y) and goal is the pose or (x,y) of the goal.
        return abs(here[0]-goal[0]) + abs(here[1]-goal[1])
    
    def findCost(self, A, B):
    # Returns the cost (integer) of moving from pose A to pose B.
    # input A: tuple of 3
    # input B: tuple of 3
    # output goes to self.cost in the appropriate cell.
    # Turns cost 2 and reversals cost 1. cost is zero otherwise.
        #print("Cost inputs: A = ",str(A),", B = ",str(B),"\n")
        cost = 0
        if B[2] != A[2]: # have to turn
            cost = cost + 2
        if A[2] == 0 and B[0]-A[0]<0: # have to reverse
            cost = cost + 1
        elif A[2] == 2 and B[0]-A[0]>0: # have to reverse
            cost = cost + 1
        elif A[2] == 1 and B[1]-A[1]<0: # have to reverse
            cost = cost + 1        
        elif A[2] == 3 and B[1]-A[1]>0: # have to reverse
            cost = cost + 1
        self.cost[B[0],B[1],B[2]] = cost
         
    def genPath(self, start, goal):
    # "Follow the arrows" back to the start
    # Inputs: start (3-list/tuple) and goal (3-list/tuple)
    # Output: path (list of 3-lists)
        path =  [goal]
        (x,y,th) = goal
        #numpies = self.cameFrom[x,y,th,:] # numpy.int64 (not python int)
        #(x,y,th) = [int(x) for x in numpies]
        (x,y,th) = self.cameFrom[x,y,th,:]
        while (x,y,th) != start:
            # while we aren't back at the start
            path.append((x,y,th))
            (x,y,th) = self.cameFrom[x,y,th,:]
            #print((x,y,th))
        return path

#class node:
#    def __init__(self,coords,parent)
#        self.coords = coords
#        self.parent = parent
#        self.children = []
