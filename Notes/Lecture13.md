# Search Algorithms

## Really Simple:

Random search: try a random direction; persist until hitting an obstacle; repeat.
Brute-force search: cover all the ground until the robot finds the goal.
*Optimal if there are no obstacles and the goal location is unknown.* So, optimal for covering a space.

## More efficient with obstacles: Uninformed searches

We'll be using variables *b*, the branching factor (avg. # child nodes per parent node in a tree diagram), and *m*, the (avg) depth of the tree.

These are called *uninformed searches* because the algorithm doesn't take into account where the goal is. (You are uninformed as to the location of the goal.)

### Depth First Search (DFS)

Go in a direction until you find either a goal or a wall.

If you see a wall: turn in a specified direction. If there is still a wall, keep doing that until you find an un-searched direction which is not into a wall.

If you reach a spot which has already been searched, treat it like a wall.

This is equivalent to drawing a tree graph all the way down one branch, then all the way down another branch, etc. That's why it's called "depth first."

Memory: there are two buffers: the frontier buffer (what hasn't been searched) and the "known" buffer.

#### Stats

* Complete if the graph is finite
* Time complexity scales with *b^m* where *b* is the branching factor and *m* is the depth of the tree. (Unless it's lucky and picks the right branch first.)
* Memory buffer is LIFO.

### Breadth First Search (BFS)

Similar to the DFS, but it tries starting all branches of the tree diagram first; then it tries the children of each node.

Memory allocation for this looks like an array with size *mn* if the space is rectangular in configuration space, *m &times; n*.

Note that we're talking about configuration space, so the physical space need not be rectangular or 2-D. 

#### Stats

* Complete if the branching factor is finite
* Finds an optimal solution. (It's looking at the precomputed map; it knows where it's heading.)
* Time complexity scales with *b^m* like the DFS
* Memory complexity also scales with *b^m*. Bad if the tree is deep.
* Memory buffer is FIFO.

### DFS vs. BFS

* Use DFS if space is restricted, but the graph doesn't have some sort of loop
* Use BFS if you must use the shortest path and some solutions are shallow, but not if the solutions tend to be deep in the tree.

### Could we use DFS or BFS in our scenario?

#### BFS

*b*=4 if we plan in *x-y* space (don't care which orientation we wind up in)
*m*=20² = 400
If each node is a float, DFS requires 6.4 KB &lt; 328KB (what the Artemis has).

Doable but not the most efficient.

#### DFS in basic form

Memory required is huge! Not doable on the Artemis (nor on most PC's.)

#### Saving the lowest-cost paths

What if we don't revisit checked nodes? Then it becomes more reasonable, but still not as good as it could be.

### Dijkstra Algorithm: Lowest-cost-first search

Assign a cost to turning and to driving straight.

This is literally the BFS, but with a bias to go in the most efficient direction.

## Informed searches

Now we are considering the location of the goal.

* Consider parent cost
* Estimate the shortest path to the goal
    * Straight line?
    * Rubber band?
    * Manhattan distance
* Much better than BFS in terms of memory
* Can get stuck in infinite loops (not complete)
* Time and space complexity same as BFS
* Not optimal: e.g. local minima

### A* Search: Holy Grail of search algorithms

Search like BFS, but priority is determined by costs (like Dijkstra) and a heuristic telling ~~how far~~ what this cost is to reach the goal.

Biased in the direction of the goal (where the highest priority is.)

### But how do we get a good heuristic?

* Straight line to goal: too optimistic -> *admissible*
* Too pessimistic -> *not admissible*
* Just right: probably lots of computing power

### A* Specs

* Complete for a finite size (based on BFS)
* Same time and space complexities as before, but usually it uses less memory and time
* Optimal, if the heuristic is admissible!

> If you want to make big money at Google, get really good at making heuristics.
> They often use slightly inadmissible heuristics since they run faster; and run them more times.