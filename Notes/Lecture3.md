# Lecture 3: Sensors

09/15/2020

There will be a few topics in this lecture.

## Lab Workflow

Each lab has two steps:

1. Python programming
	* Simulation
	* For now: all onboard
	* Later: offload some of it to the PC

2. Arduino programming

# Sensors

Some history: "Robots" (automatons) have existed for a very long time, but the first "real" robot was Shakey (1972).
It had motion planning, sensors, and a camera. All these things were expensive!

## Types of Sensors

* Do they sense stuff inside the bot or outside it?
	* Proprioceptive: Motor speed, wheel load, joint angles, batt voltage
	* Exteroceptive: Distance measurements, light intensity, sound ampl.
* Do they require a power source?
	* Passive
	* Active
* What type of sensing (i.e. on the human body) are they equivalent to?

## Some important characteristics

* Range
	* Dynamic range (decibels)
* Sensitivity

> I have this resolution, but my sensor might not respond unless there's at least this much change in the signal.

* Precision, Accuracy, and Resolution
	* Accuracy: How close is the measurement reading to the real value?
	* Precision: Given the same value, will I always get the same measurement?
	* Resolution: How small a discrete step can you measure?
* Power consumption
* Size
* Sampling rate
* Protocol

## Distance Sensors

Once upon a time, most distance sensors were ultrasonic. Now, that's not the case.
IR vs. sound:
* IR has around 2° FOV, vs. 15° or more for sound
* IR is less echoey
* Light travels faster

* Infrared (what we're using)
	* Amplitude-based: inverse-square law to measure distance
		* Very cheap
		* Very simple circuitry
		* Good for
			* Object or no object
			* Break-beam sensors
			* Grayscale intensity at a fixed distance
			* Short-range distance sensor
		* Too sensitive
			* Color
			* Texture
			* Ambient light (can be compensated)
		* Our sensor: VCNL4040
			* $7
			* 20cm range (8in)
			* Ambient light sensor (compensated, but not great)
			* Programmable duty cycle
		* Sunlight wavelength has a shallow peak near 400-500 nm
			* Deep valleys based on ozone layer absorption
			* IR emitters try to fit their peaks into valleys in the sunlight spectrum
	* Triangulation
		* Find the angle of bounce (based on amplitude still)
		* Cheap and simple
		* Mid range: 5cm - 1m (2in-1yd)
		* The curve is not monotonic, so you want to know if something is right in front of you or not
	* Time of Flight (esp. ours)
		* How long does the light take to return? $r = t*c/2$
		* Mostly insentitive to texture
			* Most surfaces you encounter will be Lambartian.
			* Some are retroreflective.
			* If you have a very shiny surface it will behave differently.
		* It still depends on ambient light
		* It tells you how likely the result you got is to be right
		* Single-photon avalanche diode (for small amounts of light; no big laser)
		* Programmable field-of-view
		* Programmable timing budget: choose fast or accurate measurements
	* Close to what we see on autonomous cars (LIDAR)
		* Spinning mirror (not a laser on a slip ring)
		* Can measure ToF and intensity at the same time (texture or reflectivity)
* Ultrasonic (SONAR)
	* Higher frequencies give better resolution and lower range
	* *Insensitive* to color, texture, glass, fog, dust, smoke 😊

## Odometry Sensors

* Encoders: Evenly-spaced things on a wheel. Count how many times it turns.
	* Technology
		* Magnetic (e.g. Hall Effect)
		* Optical
		* Inductive, Capacitive, Laser
	* Use
		* Absoute
		* Incremental
	* We don't need to add encoder modules to have an encoder. Count wheel turns.
	* Dead (Deduced) Reckoning
		* Map the present state and wheel encoder measurements to the new robot state
			* Easy to implement
			* Errors integrate and grow unbounded
				* Slipping
				* Variation in the contact point of the wheel
				* Unequal wheel diameters (small differences matter!)
				* Limited resolution
			* *How do wheel errors propagate into positioning errors?*
		* Start at previous position $X_{t-1}$. Move the wheels by $\Delta s_r$ and $\Delta s_l$. What is the current position $X_t$?
			* Assume the robot travels along a circular arc of constant radius (maybe infinite.)
			* We know the radii of the wheels and their position on the robot.
			* We have formulas for the arc length for both wheels and of the circular arc.
			* Distance traveled (arc length) is the average of the two wheel rotations (also arc length).
			* Angle traveled is calculated by the arc length divided by the circular radius $R = \frac{2L\Delta s_l}{\Delta s_r - \Delta s_l} $. The angle, $\Delta \theta$ is the difference between the wheel rotations divided by double the width.
		* Error propagation
			* As long as you're going straight, the error propagation is linear
			* You don't know the error; consider a normal distribution
			* Cloud of possible positions grows as you go
			* When you turn, the uncertainty gets a lot worse.
		* Moral of the story: dead reckoning gets worse the farther you go.

## Sensor Fusion

More on this next week!