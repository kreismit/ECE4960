## Maps and Path Planning

### 3 types of maps

* Simple grid
	* Simple and easy to compute
	* Contains lots of detail
	* Wastes lots of memory for simple objects
* K-d grid
	* Like a fractal: recursively divide into smaller squares until the resolution is satisfactory
	* More memory efficient
	* Contains lots of detail
	* Less CPU-efficient
* Topological map
	* Very CPU- and memory efficient
	* Works well if there are enough nodes
	* Minimal detail

### Path Planning

#### Optimal planning
Virtually impossible for many cases.

#### Potential Fields

* The robot follows a 
* The goal is a parabolic attractor (spring equation)
* Obstacles are inverse-square repulsors (antigravity / charged particles)
* Pitfalls: local minima
	* Long walls
	* U-shaped obstacles
* Solutions to local minimum problem:
	* Stochasticity (random noise)
	* Procedural: if(stuck) begin Bug 1

#### Graph search...

Use the configuration space!

> It's much, much easier to plan there, even though it's much, much harder to think about.

Add a third dimension to the graph for rotation.

* Exact Cell Decomposition
	* Tesselate the map exactly into contiguous free spaces.
* Approximate Cell Decomposition
	* Break the map into a grid.
	* Fixed cell decomposition: fixed grid size
	* Adaptive decomposition: like K-d
* Visibility Graphs
	* Connect visible vertices on the way to the goal (rubber band)
	* Connect visible vertices on obstacles
	* Inflate the obstacles so the robot doesn't touch them

##### Probabilistic Planners

Explicit geometry-based planners get exponentially worse as the # dimensions goes up.

Sampling-based planners don't compute the entire map; rather, they compute a local sample (e.g. Jacobian in I.K.)

* Probabilistic Roadmap
	* Compute a bunch of random points in the configuration space (forward rather than inverse kinematics) *just once*
	* Throw out the points that are colliding with obstacles
	* Connect the nearest-neighbor points
	* If each connection is collision-free, we add it to the graph.
	* Problems:
		* Non-optimal
		* Could wind up with a graph broken in pieces (not connected)
	* Other versions:
		* Single-query (load up the map once)
		* Multi-query (rebuild the map sometimes)
		* Uniform node sampling
		* Biased node sampling (cost, edges, etc.)
		* Collision detection strategies for connections (time-consuming)
	* Speeding it up:
		* GPU's
		* FPGA's
* RRT (Rapidly exploring Random Trees)
	* Robotic slime molds. Keep extending branches randomly until you reach a goal.
		* Only keep the branches which are collision-free.
		* Some planners plan from the start and from the end at the same time.
		* Some planners work with a bias, like an octopus in a maze.
	* Problems:
		* Step size too small: slooow
		* Step size too large: miss the goal or go through obstacles
		* Suboptimal
	* Variations:
		* RRT\* (converges to optimal solution; takes longer to compute)
		* RRT Connect: Start from start and from goal
		* A\*-RRT
		* And others

Check out https://pythonrobotics.readthedocs.io!