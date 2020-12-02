# Lecture 18: More on Control

## Observability

Just as controllability is a measure of how well we can achieve all possible states given some control input *u*, observability is a measure of how well we can determine the state of a system given some measurement *y*.

The LQR (Linear Quadratic Regulator) is the main means of generating an "optimal" controller; the Kalman filter is the main means of generating an "optimal" observer. (At least for our class.)

Observability matrix: $$\sigma = \begin{bmatrix} C \\ CA \\ CA^2 \\ ... \\ CA^{n-1} \end{bmatrix}$$

Iff the rank of the observability matrix is *n* (the size of the state variable) then the system is observable.

Iff the system is observable, we can estimate *x* from *y*. There is an observability Gramian, just like there is a controllability Gramian. This is determined from the SVD of $\sigma$.

    ----------->|| system ||-------------->
        |   |                       |
        |   ----->----------->---   |
        |                       |   |
        -<-----||LQR||--||KF||<---<-

Note that the disturbances could be physical events, or they could be 

## Kalman Filter

Not that much different from the Bayes filter. The Bayes filter uses the previous position, a "control" step, and an update step. It has to do lots of loops (slow). However, the Kalman filter assumes that both the posterior and the prior belief are Gaussian variables.

### Prediction Step

It has a prediction step too: given a prior state, it predicts the next step. A dynamical matrix predicts the next mean given the current mean. It then determines the uncertainty (kind of like standard deviation) based on the *same* A matrix and a process noise matrix.

$$\mu_p(t) = A\mu (t-1) + B u(t)$$
$$\Sigma_p(t) = A\Sigma (t-1) A^T + \Sigma_u$$

### Update step

First, compute the "magic" Kalman filter gain $K_{KF}$:
$$K_{KF} = \Sigma_p (t) C^T (C \Sigma_p(t) C^T + \Sigma_z)^{-1}$$

Then find the covariance matrix $\Sigma$ and the mean $\mu$:
$$\mu(t) = \mu_p (t) + K_{KF}(z(t) - C\mu_p(t))$$
$$ \Sigma(t) = (I - K_{KF}C) \Sigma_p(t))$$

### Implementation

Literally copy-paste the equations off the slides.

Inputs:

* Process noise matrix $\Sigma_u$
* Measurement noise matrix $\Sigma_z$
* Dynamical matrix $A$
* Control matrix $B$

$$\Sigma_u = \begin{bmatrix} \sigma_1^2 & 0 & 0 \\ 0 & \sigma_2^2 & 0 \\ 0 & 0 & \sigma_3^2 \end{bmatrix}, \quad \Sigma_z = \begin{bmatrix} \sigma_4^2 & 0 \\ 0 & \sigma_5^2 \end{bmatrix}$$

We can assume (for reasons we won't go into here, but it makes intuitive sense) that all the sensor readings are independent. That's why we're only filling in the diagonals.

Outputs:
* Predicted mean $\mu(t)$
* Predicted covariance $\Sigma(t)$

## Labs

The inverted pendulum isn't happening.

We have two options over the next couple of weeks, both of which involve controllers and observers. So, do lab 11a and 12a or 11b and 12b (but don't mix and match.) Lab 12 due date will be postponed.

The split between Lab 11 and Lab 12 is the same for both versions:

* Lab 11: LQR
* Lab 12: Kalman filter

### Option 1 "Turning a Corner"

Physical robot: implement full-speed wall-following and turns.

Will use first-order principles and fitting (generally lots of that).

We need to...

* Implement a controller and an observer
* Drive at full speed and follow a corner... maybe you can get it just right
* Use PID or open-loop control to control speed, using LQR to get gains...
* Use a Kalman filter to get accurate state estimates for the controller
* Will need a long, clear wall and a "cushion" for when it hits the wall
* Recipe...
    * State-space control
    * Equations of motion
    * Estimate parameters for A and B
    * Estimate & tune parameters forf Q and R
    * State space: distance from the wall, *z*, and an angle, *θ*
    * Assume one motor (or the other) is always on at full power
    * Step response and system identification: is that accurate?
        * Second-order system due to inertia
        * Can treat theta dot instead of theta as first-order (add another line to the state)
    * $\frac{U}{I} - \frac{d}{I}\dot{\theta} = \ddot{\theta}$ - need to ID the sys
        * Steady-state: $\ddot{\theta} = 0$ -> estimate $d$
        * ~~Zero rotational velocity: $\ddot{theta} = \frac{U}{I}$ -> estimate $I$~~
        * Use rise time ($y = 1-e^{-t/\tau}$ given the form $\frac{dy}{dt} + \frac{1}{\tau}y = x$)
* What's left?
    * Implement LQR feedback (find $Q$ and $R$ cost functions)
    * Implement Kalman filter to estimate true values (the ones you can't measure directly)

> Ramp it up to full speed first, and *then* trust your model.

### Option 2

Simulated robot: implement inverted pendulum on a cart.

Will use first-order principles.

Maybe safer if the robot isn't working great yet.

How do you best use simulations?

* Quick and safe testing
* Check implications of nonlinearities (can you simulate them?)
* Check model parameters against realistic ones

> It's not just for pretty plots, and so on - it's a very powerful framework to test how sensitive our controller will be before we break anything.