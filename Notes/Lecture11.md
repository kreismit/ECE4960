# More on Localization

In each of these cases, we assume we *know* the map already, and we don't know where the robot is.

## Likelihood Model

In many ways, this is similar to the beam model. However, it doesn't use physical explanations, but rather normally distributed probabilities of sensor ranges. 

It requires a map and a guess of the robot's pose (from odometry). It finds the probability of a certain measurement the robot took.

> Whenever you think about an algorithm, you should start by considering the inputs and the outputs.

Probability distribution is calculated similarly to the beam model: sum the 2-D Gaussian distributions around each obstacle on the map, the uniform distribution of $z_{max}$, and another distribution for random failures.

It is less computationally intensive than the beam model since it does everything in 2-D (doesn't care about angles) and it's simpler.

However, it doesn't take into account physical boundaries or unexpected objects.

## Feature-based Models

If you get lost, you can find your way again using landmarks. Feature-based models are based on the same idea - landmarks or *features*. They are extracted from *dense measurements* rather than the raw distance measurements (like those above.)

We will focus on trilateration and triangulation from landmarks.

### Trilateration and Triangulation

e.g. RoboCup. The robots determine their position on the map using six differently-colored landmarks in different corners/edges of the field.

*Trilateration* is localization using *distances* from three points. *Triangulation* is localization using the *angles* of ranging from three points. (If you have both distance and angle, you don't need three points anymore.)

## Summary of Sensor Models

* They *explicitly model uncertainty.* That's key.
* Good models can be found using a simple procedure.
* Motion models can, too.
* **Be aware of the underlying assumptions of the model.**

## Motion Models

Robot motion is inherently uncertain. Sensors drift. Range measurements are imprecise. Odometry doesn't work when the robot is imperfect and the robot is not traveling on a perfectly flat, smooth surface. There are lots of vibrations and noise sources in the environment.

So, we need a probabilistic model for motion too. We'll model the probability of $x_t$ given the last state, $x_{t-1}$, and the action taken, $u_t$.

## Velocity Model

Let's say we know the position and angle of a robot at the start and end of a short travel. There are many different arcs than connect the points, but only one matches the starting and ending angles.

It might be hard to control the angles precisely due to drift, but it's easy to control the arc using the *angular velocity* during the travel.



Special cases:

* Radius is zero: the robot spins in place.
* Radius is infinite: the robot travels in a straight line.

Now, we will take into account the angle. The exact motion may be calculated using the *velocity* and the *angular rate*.

This will be probabilistic, however; our $v$ and $\omega$ will have a certain mean and standard distribution. We can compare the values we get from the motion model with the values we get from the sensor model given the map.

$$u_t = (v_t,\omega_t)$$

where both $v_t$ and $\omega_T$ are normally distributed.

Note that sometimes we use a *triangular distribution* rather than a Gaussian (normal) distribution (quick approximation.)

> Often in probabilistic motion models, we always look at zero-mean Gaussians because they're easy.

The mean is added as an offset later.

Now, flip this on its head. We measure the translational and angular velocities and try to calculate the position at the end of travel.

We will get some sort of cloud as a result of this *velocity model*. This doesn't take into account the map, but we can combine it with the sensor model and the map data to get a better localization.

## Odometry

Use dead-reckoning position and angle measurements instead of velocities. The algorithm is quite similar.

The odometry is more accurate, but it doesn't allow us to predict.

Q: Why is the odometry model more accurate?
A: The velocity model assumes you're moving in an arc of a circle. You are never moving in a perfect arc.
However, the odometry model is more computationally expensive (?).

Usually, *the velocity model is used for path planning and odometry model is used for prediction.*