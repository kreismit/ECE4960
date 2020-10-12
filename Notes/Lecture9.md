# More on Localization

Vivek Thangavelu

## Robot Belief

The *belief* of a robot is the posterior distribution over the state of the environment, given all past sensor measurements and all past controls. They are conditional probabilities, conditioned on the prior data (but not the current data!)

## Bayes Filter Algorithm

Here is the pseudocode.

Inputs:

1. for all $x_t$:
2. $$ \overline{bel}(x_t)=\sum_{x_{t-1}}p(x_t|U_t,x_{t-1})bel(x_{t-1})$$
3. $$ bel(x_t)=\eta p(z_t|x_t)\overline{bel}(x_t) $$
4. return $bel(x_t)$

### Line 2: Prediction

We actually *don't* incorporate any measurements; only the previous state (belief) and the action taken. This term on the second line, $p(x_t|u_t,x_{t-1})$ is called the *transition probability* or the *action model*. It predicts how the robot will "transition" to the next state given the last state and the action taken. It's a dynamical model. 

### Line 3: Update/Measurement

But still we haven't incorporated the measurement data. So, we'll take the probability of getting a measurement given that we're in a certain state, $p(z_t|x_t)$. It's called the *measurement probability* or the *sensor model.* This whole thing is multiplied by the output of the last step to get a probability. There is a certain multiple $\eta$, called the *normalization constant*; this ensures that the result is still a probability (between 0 and 1.)

## Example 1

A ~~springloaded~~ door is either open or closed. A simple robot can sense its state and can either push it or do nothing.

	x = {is_open, is_closed}
	z = {sense_open, sense_closed}
	u = {push, do_nothing}

	p(z_t == sense_open | x_t == is_open) = 0.6
	p(z_t == sense_closed | x_t == is_closed) = 0.8

	p(z_t == sense_open | x_t == is_closed) = 0.4
	p(z_t == sense_closed | x_t == is_open) = 0.2

	p(x_t == is_open | x_{t-1} == is_open, u_t == push) = 1
	p(x_t == is_open | x_{t-1} == is_open, u_t == do_nothing) = 1
	p(x_t == is_closed | x_{t-1} == is_closed, u_t == do_nothing) = 1

To do the Bayes filter, we must know what the initial state is. Begin the loop.

In each loop, we sum all the probabilities that $x_{t}$ will be something, given what our beliefs were regarding $x_{t-1}$ and $u_t$. Then, we adjust this with the measurement step and normalize it.

## Example 2

A robot can be in any of six positions along a number line: 0-5. It can move one position at a time.

The probability that the robot actually moves +1 when it tries is 0.5; similarly, the probability that it actually moves -1 when it tries is 0.5.

Also, given a sensor reading, we are 50% correct, 25% one too far to the right, and 25% one too far to the left. If we're at the right edge, however, the probabilities must be normalized because there is now 0% probability of being past the right edge.

Our initial knowledge is nothing, so the initial distribution is uniform. The probability of any state is 1/6.

Our first measurement is a wall on the right. Thus, after normalizing, our probability of being in state 5 is 2/3; our probability of being in state 4 is 1/3.

Then, we drive one square to the left. We must do the sum:

1. If we were in 5 (2/3 prob.) then we are 50% likely to be in 4 and 50% likely to be in 5.
2. If we were in 4 (1/3) prob.) then we are 50% likely to be in 4 and 50% likely to be in 3.

The belief for this state is then 1/6 for square 3; 1/2 for square 4; 1/3 for square 5.

What if we were certain it was at 0 to begin with, but the sensor senses 5? Then, we don't get a valid probability distribution. But, if we assign a very small probability to anything besides 5, then we still get a valid dist. at the end.