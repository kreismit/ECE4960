# Lecture 5: Feedback Control

## Why we need feedback loops

Disturbances.

## PID Control

### P Control

This lecture is heavily inspired by the Matlab Tech Talk: Understanding PID Control.

$$u(t) = K_Pe(t)+K_I\int{0^te(t)dt+K_d\frac{de(t)}{dt}$$

	set point/
	reference/
	actuation signal +										output/
	----------------->O---->||Kp||-->||Plant||-----------> controlled variable
					- |_____________________________|

Our controller operates on the *error*, the difference between setpoint and output. $e=r-y$.

Examples:

* Walking to 50-ft mark on football (or soccer) field
* Flying a drone to 50 m height

When there's a disturbance force (e.g. gravity) then you will have a steady-state error. The higher your gain (if you don't saturate) the smaller your steady-state error is.

### Getting Rid of Steady-State Error

Give the P controller some notion of the past.

			---->||1/s||--->
		+	|			  + | +
	--->O--------||Kp||---->O----->||Plant||---------> output
	    | -										|
		----------------------------------------

This PI control brings the steady-state error to zero. In industry, most PID controllers are actually PI controllers.

BUT... the integrator isn't infinitely fast. Neither are our sensors. So our sufficiently large integral term will oscillate somewhat as it overcorrects again and agin.

### Getting Rid of Integral Overshoot

Predict the future too.

			---->||Kd*s||---
			|				|
			---->||Ki/s||---
		+	|			  + | +
	--->O--------||Kp||---->O----->||Plant||---------> output
	    | -										|
		----------------------------------------

The derivative term linearly extrapolates into the future so that as we close in on the setpoint, if we're moving too fast, the derivative slows the thing down so that we don't overshoot.

We get to control these three gains: $K_P$, $K_I$, and $K_D$. For most systems, this is sufficient.

But, we still have issues.

* Integral can cause overshoot.
* Systems aren't linear.
	* Actuator
		* Backlash
		* Brake constraints
		* Saturation
	* Process
		* Suppose I hold on to the drone for a bit as the controller is starting up. The integral overshoots.
		* Suppose the controller is saturated, but the integrator is still going. Then the integral will wind up.

### Troubleshooting the PID Controller

#### Integral Clamping

* If the output is unable to increase any more, then we know we've saturated. This is implemented by comparing the sent signal to the actual signal (or comparing the calculated signal to max and min.)
* If the actuator is saturated, send zero error to the integrator *only* if the integrator output and the error have the same sign.
* This limits the overshoot and prevents integral windup.

#### Filtering

* Real measurements have noise of some kind.
* Derivatives amplify HF signals more than LF signals.
* A low-pass filter undoes this.
	* First-order: $\frac{N}{s+N}$
* Better: You can add an integrator in the feedback path instead of a derivative in the feed-forward path!
$$y = N\left(u-\frac{y}{s}\right)$$
$$ y + \frac{Ny}{s}=Nu$$
$$ y = \frac{N}{1+\frac{N}{s}}u$$
So, the transfer function is
$$\frac{y}{u}=\frac{N}{1+N\frac{1}{s}}$$
which looks just like the derivative.

As it turns out, the integral in the feedback is *faster* than the derivative in the feed-forward path. Then, this tends to improve noise tolerance.

### How to Tune a PID

#### Look at the system.

Is it well-behaved? If not, PID controllers aren't good to use.

#### The system isn't crazy. Then what?

* If well-behaved, you can either
	* Test and design
		* You get accurate test results
		* You might be worried about breaking the physical system
	* Model-based design
		* The model could suck
		* The model might be cheaper than breaking something
		* The model could give you a good starting point
* Many options.
	* Test first to get first principles; then use model-based design
	* Run an input sequence (e.g. step input).
		* Use heuristics to get gains.
		* Calculate system parameters and calculate gains.
		* Use special software.
	* Start with a good model and use auto-tune software.
	* You will still have to tweak things regardless of how good your model or software is.

#### Example

We want to write a PID to make the robot turn accurately.

Use the state-space equation

$$\dot{x} = Ax+Bu$$

$$F = ma; \qquad \tau=I\alpha=I\ddot{\theta}$$
$$ u=\dot{theta}c=I\ddot{theta}$$
$$ \ddot{theta}=\frac{-\dot{theta}}{I}+\frac{1}{I}u$$
That's our equation of motion.

Back to our state space.
$$x=\left[ \theta \atop \dot{\theta} \right]$$
$$\dot{x} = \begin{bmatrix} 0 & 1\\ 0 & \frac{-c}{I} \end{bmatrix}x + \left[ 0 \atop \frac{1}{I} \right] u$$

The result is some code (a Jupyter notebook) which has a tinyurl in the slides. The link is also [here](https://colab.research.google.com/drive/1F-iZnkXpKLOP6C4EZoSbmBm68C002yBk?usp=sharing).