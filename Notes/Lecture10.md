# Lecture 10 - Beans, Probability, and Sensor Models

## Beans

Each row of cups was a new probability array.

You need three arrays:

1. Initial state
2. Intermediate (after moving, but before measuring)
3. Post-measurement

#3 becomes the #1 for the next iteration.

(These are probably not named correctly. You get the idea.)

## Sensor Models

#### Sensors are unpredictable.

Even if you have a wonderful model and very precise, expensive sensors, the model you built for one room might not work in the next room. (I speak from experience that this is true. "But it worked at home!")

#### Gaussian Distribution

Also called the normal distribution or a bell curve. It depends on two parameters: the mean μ and the standard deviation σ.

There is such a thing as a 2-D Gaussian distribution. It's round if the two standard deviations are the same; if they aren't, it has elliptical contours. So, there are three parameters needed.

**Probabilistic models are robust to lots of things you can't predict.**

### Beam Model

Condition the robot on a map, which is an abstraction of the real physical world. 

Take a bunch of measurements at one time (interval) *t*.

There are four types of measurements we can get; three of them are errors.

#### Measurement Errors

All ranging sensors have a minimum range and a maximum range. If the beam never bounces back, it just gives you the maximum range.

1. Correct measurements
	* Let $z_t^{k*}$ denote the true range, and $z_t^k$ be the individual measurement.
	* The $z_t^k$ should be normally distributed about the mean $z_t^{k*}$.
	* Get the true measurement based on the map and ray-casting (draw lines between different points to find the average in the middle.)
	* If $z_t^k$ is greater than $z_{max}$ (maximum possible range) or less than then the minimum range, then the probability of that range is zero.
2. Unexpected objects
	* Objects not contained in the map produce unexpectedly short ranges.
	* If multiple objects may appear with equal likelihood, the sensor is more likely to see the nearer ones since the ray doesn't go through them.
	* Model the probability of too-short ranges as an exponential distribution which drops off to zero at the actual range $z_t^{k*}$.
3. Failures (no returned beam)
	* A SONAR sensor can miss an object because of specular reflections (the angle is too sharp)
	* A laser sensor can fail when an object is sufficiently black
	* There is a maximum possible range it can give. The likelihood of *never seeing a return signal* is given by a spike at (or near) the maximum possible range.
4. Random measurements: lumping all other possible causes into a uniform distribution.

At the end, sum all the distributions. You get a left-skewed (rising tail) normal distribution plus a spike at the max range.

This algorithm requires several parameters: one for each distribution, and also a &lambda; for the short-range distribution. They can be estimated by...

* Guesstimate
* MLE
* Hill-climbing, gradient descent, genetic algorithms, ...

This model is pretty good. It has limitations, however.

* Overconfident: assumes independence between beams.
* Needs different models for different reflection angles
* Ray-tracing is computationally expensive (but this can be pre-processed)
* Not good for small stuff, edges, or clutter

### Other models next week!