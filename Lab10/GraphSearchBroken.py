#!/usr/bin/python
import numpy as np

class search:
    def __init__(self, map):
        # Input: map (2-D numpy array, grid generated previously)
        self.map = np.stack([map,map,map,map],axis=-1) # map is the same for all angles
        self.maxRow = np.shape(self.map)[0]
        self.maxCol = np.shape(self.map)[1]
        self.reached = np.zeros([self.maxRow, self.maxCol]) # lookup for reached cells
        self.solution = None # have we found a solution? if so, this is it
        self.path = []  # this list contains paths that have been tried

    def neighbors(self, node):
        # Find neighbors in a 3-D graph
        # Input: node (list or tuple of 3)
        # Output: out (list of tuples of 3)
        #directions = [[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]]
        # Can't drive sideways.
        if node[2] == 0 or node[2] == 2:
            directions = [[1,0,0],[-1,0,0],[0,0,1],[0,0,-1]]
        elif node[2] == 1 or node[2] == 3:
            directions = [[0,1,0],[0,-1,0],[0,0,1],[0,0,-1]]
        else:
            raise Exception("Out-of-bound rotation")
        neighborList = []
        out = []
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

    def frontier(self,node):
        # Determine which cell neighbors are the frontier
            out = self.neighbors(node)
            for i in out:
                if self.map[i[0],i[1],i[2]] is 1:
                    out.__delitem__(i)
            return out

    def dfs(self, here, goal, branch = 0):
        # Depth-first search (with cost)
        # Inputs: here (1-D numpy array of 2 or 3 coordinates)
        #         and goal (1-D numpy array of 2 coordinates)
        # Output: list of nodes leading to goal
        if len(here) is 2:          # if the input is just [x,y] or (x,y)
            here = list(here)       # make sure it's mutable
            here.append(0)          # add an angle coordinate
            here = tuple(here)      # and make it immutable to avoid problems
        if len(goal) is 2:          # if the input is just [x,y] or (x,y)
            goal = list(goal)       # make sure it's mutable
            goal.append(0)          # add an angle coordinate
            goal = tuple(goal)      # and make it immutable to avoid problems
        self.reached[here[0],here[1]] = 1   # been here!
        if branch is 0:             # currently on the root branch!
            self.path = []
            self.path.append([here])  # initialize path variable
            self.solution = None    # this means we aren't at the goal either
        #print("Branch #{}".format(branch)) # for debugging
        #print(self.path[branch])
        next = self.frontier(here)
        # Loosely sort in order of best heuristic and cost.
        bestPriority = 1000
        for i in range(len(next)):
            priority = self.cost(here,next[i])
            if (priority < bestPriority):   # better than the best so far!
                bestPriority = priority     # this is the new best
                next.insert(0,next.pop(i))  # move it to the front.
        # as long as the frontier isn't empty & the algorithm isn't done
        if not not(next) and self.solution is None:
            nextBranch = branch
            for i in next: # i is a set of coordinates in list next
                #print(i)
                if not self.reached[i[0],i[1]]: # only if unreached
                    #print("Trying cell ({},{},{}) next".format(*i))
                    self.path.append(self.path[branch])
                    nextBranch = nextBranch + 1
                    self.path[nextBranch].append(i)
                    if i[:] == goal[:]: # we're done!
                        self.solution = self.path[branch]
                        print("Done!")
                        print(self.solution)
                    else:
                        self.astar(i,goal,nextBranch) # start next recursion

    def astar(self, here, goal, branch = 0):
        # A* search
        # Inputs: here (1-D numpy array of 2 or 3 coordinates)
        #         and goal (1-D numpy array of 2 coordinates)
        # Output: list of nodes leading to goal
        if len(here) is 2:          # if the input is just [x,y] or (x,y)
            here = list(here)       # make sure it's mutable
            here.append(0)          # add an angle coordinate
            here = tuple(here)      # and make it immutable to avoid problems
        if len(goal) is 2:          # if the input is just [x,y] or (x,y)
            goal = list(goal)       # make sure it's mutable
            goal.append(0)          # add an angle coordinate
            goal = tuple(goal)      # and make it immutable to avoid problems
        self.reached[here[0],here[1]] = 1   # been here!
        if branch is 0:             # currently on the root branch!
            self.path = []
            self.path.append([here])  # initialize path variable
            self.solution = None    # this means we aren't at the goal either
        #print("Branch #{}".format(branch)) # for debugging
        #print(self.path[branch])
        next = self.frontier(here)
        # Loosely sort in order of best heuristic and cost.
        bestPriority = 1000
        for i in range(len(next)):
            priority = 2*self.heuristic(next[i],goal) \
                     + self.cost(here,next[i])
            if (priority < bestPriority):   # better than the best so far!
                bestPriority = priority     # this is the new best
                next.insert(0,next.pop(i))  # move it to the front.
        # as long as the frontier isn't empty & the algorithm isn't done
        if not not(next) and self.solution is None:
            nextBranch = branch
            for i in next: # i is a set of coordinates in list next
                #print(i)
                if not self.reached[i[0],i[1]]: # only if unreached
                    #print("Trying cell ({},{},{}) next".format(*i))
                    self.path.append(self.path[branch])
                    nextBranch = nextBranch + 1
                    self.path[nextBranch].append(i)
                    if i[:] == goal[:]: # we're done!
                        self.solution = self.path[nextBranch]
                        print("Done!")
                        print(self.solution)
                    else:
                        self.astar(i,goal,nextBranch) # start next recursion
    
    def heuristic(self, here, goal):
    # Just Manhattan distance.
    # here is the current pose (or x,y) and goal is the pose or (x,y) of the goal.
        return abs(here[0]-goal[0]) + abs(here[1]-goal[1])
    
    def cost(self, A, B):
    # Returns the cost (integer) of moving from pose A to pose B.
    # A and B may be 2- or 3-vectors.
    # Turns cost 2 and reversals cost 1. cost is zero otherwise.
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
        return cost
         
    

#class node:
#    def __init__(self,coords,parent)
#        self.coords = coords
#        self.parent = parent
#        self.children = []
