# ECE 4960 Lecture 1
09/03/2020
Kirstin Petersen

* Recommends speaker mode
* Recommends asking questions

### So... why should you take this class?
* Other classes have slow-moving robots
* Fast robots are fundamentally different from slow robots
	* Slow robots are "quasistatic"
		* If you stop the robot, it will probably keep standing
		* Kinematic regime (you don't need to take into account inertia)
	* Fast robots require good build and good understanding of dynamics
	* Fast doesn't mean fast-moving robots as much as robots that...
		* require dynamic control
		* tend to be unstable
* Other (especially undergrad) classes tend to gloss over messy details
* More than just control theory and dynamics
	* Practical implementation
	* Mechanics
	* Sensors
	* Processing
	* Estimation

### Control and Fast Robots
* Passive control
	* Simple 👍
	* Requires no active elements 👍
	* Example: semi with aerodynamic analysis
	* Doesn't work for systems which tend to be unstable 👎
* Active control
	* Example: balancing an inverted pendulum
		* You could use open-loop control: shake it up and down
			* Simple 👍
			* Inefficient 👎
		* You could use closed-loop control
	* Open-loop control
	* Closed-loop control
		* Requires no controller 👍
		* Simple 👍
		* Not error-tolerant 👎
		* Not as efficient 👎
	* Open-loop-control
	* Concerns
		* Efficiency
		* Noise & measurement error
		* Disturbances
		* Stability
	* Block diagrams
	* The feedback loop isn't everything - you need a planner too.

### Class Structure

1. Implement the robot.
	* Start with an RC car kit (tomorrow!)
		* Tank drive with geared sides
		* Cheap and has imperfections
			* No omni wheels, so steering involves skidding.
			* Fast and has momentum: when you stop, it doesn't.
			* Motors aren't fast enough to start and stop it abruptly.
	* Add a processor, sensors, and drivers (all from SparkFun)
		* Artemis! Custom libraries.
		* Motor driver boards (solder-free)
		* ToF (laser and IR) sensor
		* Prox sensor
		* IMU
	* Pros and cons of modalities and sensor types
	* Noise, bias, and sampling frequency
	* Simulation platform (thanks Vivek!)
		* Sensor models
		* Motion models
		* Inverted platform thing (thanks Sadie!)
2. _Try_ to enable fast navigation.
	* We have Vivek's cool simulator!
	* Open-loop nav
	* Obstacle avoidance
	* PID control
	* Map
	* Localization
	* Trajectory planning
	* Linearize system and find equations of motion
		* $x,y$
		* $ \dot{x} = cos(\theta)v$
		* $ \dot{y} = sin(\theta)v$
		* Use a state-space representation (look familiar from Control Systems?)
3. Control and unstable system.
	* Whee! I just transformed into Demolishor the balancing wheel monster.
	* Inverted pendulum model
	* Lots of stuff I don't know about yet

### Course Objective
* Somewhere between
	* a Culminating Design Experience (learn through implementation)
	* a foundations course
* Somewhere between
	* ECE
	* MAE
	* a little bit of CS
* Overlaps with
	* AMR
	* Feedback Controls

We have three TA's!

* Vivek Thangavelu
	* Pronounced: "we wake"
	* Has done lots of cool projects
	* Taught a class on SLAM!
* Sadie Cutler
	* Graduate student in Prof. Kirstin's lab
	* Has worked with Kirstin on drones for agriculture
	* MAE student
	* Knows German and Jiu Jitsu
	* Also has done cool robots
* Alex Coy
	* Has done lots of cool projects
	* "Spent lots of my life just getting stuff to work"
	* Studies ECE
	* Built stuff from all kinds of boards and knows OpenSCAD
	* [Personal website](https://alexcoy.duckdns.org)
	* Focuses on low-level stuff while the other two focus on high-level
	* Has tried Ubuntu and Arch but prefers Debian
* Professor Kirstin Petersen
	* Goes by Kirstin
	* Heads up [Collective Intelligence Lab](https://cei.ece.cornell.edu)
	* Has worked with
		* Swarm robots
		* Soft robots
		* Jumping robots
		* Honeybee Robotics, Inc.
	* "we know our robots"

Labs will be individual

### _Tentative_ Schedule
1. Intro (today)... GitHub
2. Rigid-body transformations... [Artemis Nano](https://www.sparkfun.com/products/15443)
3. Sensors... Bluetooth
4. Noise, prob, est... Characterization
5. PID, mapping... Open-loop control
6. Bayes filter, odometry... Obstacle-avoidance
7. Localization... IMU
8. Planning... Odometry
9. Linear sys and state space... PID control
10. Inverted pendulum dynamics... Mapping
11. Controllability (LQR)... Localization
12. Observability, Kalman filter... Planning
13. Kalman filter
14. LQG control on inverted pendulum
15. Guest lectures
16. Recap

There is no homework for this class, but there's a lab every single week.
Labs will take as long as homework.
Use GitHub!
This will be messy, and some labs won't work
You'll get to build up an online portfolio
We'll aim high

### Logistics
* Most everything is on [GitHub](https://cei-lab.github.io/ECE4960)
* Canvas (per university requirements)
* CampusWire instead of Piazza
* Lab kit to be picked up tomorrow (or mailed if you're off campus)
* You'll need tools
	* 1.5mm (ish) flatblade screwdriver
	* Wire cutters
	* A flat surface
* And software
	* Works on all three main OS types
* ~~Homework~~ Labs will be "submitted" through your personal GitHub room
* If you really need space on campus rather than your room, PH427 is available with limits
* *When* something breaks
	* Contact the staff ASAP
	* Can try on a TA kit, with limited time
	* Can get partial credit for making it work in a simulation
* **The car has very limited battery life.**
	* Runs out after 10-15 min.
	* Two batteries in each kit.
	* Spread the lab time over multiple days.
* Grading is up online...
	* 85% labs
		* 65% solution	
		* 25% write-up
		* 10% speed
	* 10% quizzes
	* 5% participation
* Collaboration is encouraged, but you must implement it yourself
* First assignments:
	* Pcik up your lab kit by Fri 9/4 at 16:00
	* Make a Github repo and build a page by Tues 9/7 08:00 
	* Upload your write-up of Lab 1...
* More on the Artemis Nano
	* **3V board**
	* Support forum on [SparkFun](https://forum.sparkfun.com)