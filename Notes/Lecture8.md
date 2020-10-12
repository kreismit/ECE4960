# Localization I (Vivek)

## Vivek

* Is kind
* Has taught about localization before
* Knows what he's talking about

## Probability in Robotics

### Review of Probability Principles

* Random variables are variables whose values depend on something we don't understand.
* Random variables are written as capital letters in probability math.
* Probabilities have restrictions:
	* The probability that a random variable equals anything it can equal is **1**.
		* Finite number of possibilities: sum probabilities of all *x*'s
		* Infinite number: integrate over *x*
	* Probabilities may not be negative.
* Joint distributions are probabilities that *X=x* and *Y=y*.
	* If we already know the value of *y*, then the probability is written as $P(X|Y)$ and is called the probability of *X* given *Y*.
	* The chain rule of probability / the basic law of proability says that $P(X|Y) = \frac{P(X=x \cap Y=y)}{P(Y=y)}=\frac{P(x,y)}{P(y)}$.
* Two events are independent iff the occurrence of one does not affect the probability of occurrence of the other.
* If events aren't really independent, they can be *conditionally independent*.
	* $p(x,y|z) = p(x|z)p(y|z)$.
	* The probability can be written with a capital or a small *p*.
* Bayes' Theorem: $ p(x|y) = \frac{p(y|x)p(x)}{p(y)}$
	* This is often used with discrete events, but it also works with probability distributions.
	* $y$ is often called the *data* in this case.
	* $p(x)$ is the *prior* probability distribution, the knowledge that we have of *x* before we incorporate *y*.
	* After incorporating the data, we get the *posterior* distribution.
	* $p(y|x)$ is called the *likelihood* because it describes how likely we are to have data *y* given *x*.

### Why use probability in robotics?

1. Environments: real world vs. factory floor
2. Sensors: limited and noisy.
3. Actuation: not made perfectly; slip and degrade.
4. Models: inherently inaccurate.
5. Computation: the robot can't think forever.

> A robot that carries a notion of its own uncertainty and that acts accordingly is superior to one that does not. - *Probabilistic Robotics*, Thrun, Burgard, & Fox

Probabilistic robotics tend to be more reliable. Example: Shakey the Robot (DARPA)

## Robot-Environment Model

### Motivation

The robot interacts with its environments in two ways:

* Sensing/Observation/Measurement
* Acting

The robot is a dynamical system that possesses its own internal state. It can acquire information about its environment using its sensors; but sensors are noisy and can't sense all aspects of the environment directly. Thus, the robot maintains a belief with regards to the actual state of the environment, which could be even farther off than the measurements. Lastly, the actuators are inherently uncertain. So the robot could do anything. Right?

### Model Basics

Model it as a discrete-time system.

The robot's *state* at time $t$ is denoted $x_t$. A *sensor measurement* at time $T$ will be denoted as $z_t$. A control action will be denoted $u_t$. It carries information about the change from one state to the next.

For no particular reason (well, it affects the math) the robot executes a control action $u_t$ first, and then takes a measurement $z_t$. Exactly one control action occurs at any time $t$. One possible action is "do nothing."

### State $x_t$

This is a *state variable* as you know it from Dynamics.

Could be:

* Robot pose: location and orientation
* Robot configuration: travel of each actuator
* Robot velocity and joint velocities
* Location and features of surrounding objections

### Sensor Measurements $z_t$

Each measurement [can] improve our the robot's knowledge of its state.

### Control Actions $u_t$

Carries information about the change in the robot's state. Each action induces more noise.

### Probabilitistic Generative Laws

How are $x_t$ and $z_t$ generated stochastically?

#### State Generation

$x_t$ is stochastically generated from $x_{t-1}$, and it depends on

* All steps throughout times 0 to $t-1$
* All measurements from times 1 to $t-1$
* All actuations from times 1 to $t-1$

$$p(x_t|x_{0:t=1},z_{1:t-1},u_{1:t-1})$$

that is, the probability of $t$ given $x, y, z$ at previous time steps.

How in the world do we keep track of *all* past events?

## Markov Assumption

**Past and future data are independent if one knows the current state.**

This does *not* mean that the future is a deterministic function of the current state. Also, the Markov assumption doesn't hold for all models.

A *stochastic* model is one that depends on the Markov model. Thus, it lumps past data into a current state that can be used to predict the future.

### Markov Process 1: The Drunkard's Walk

The drunkard doesn't remember where he has been. His walk is random. The position may change by +1 or -1 with equal probability.

The state $X_n$ is the position along a number line.

If someone tells the drunkard he is at position 3 on the number line, that is no more information than that the drunkard has been at position 2, 3, 4, and then 3 again.

### Markov Provess 2: Drawing Coins

Now, $X_n$ is the total value of the coins set on the table. Each step is drawing another coin from a purse.

Let's say we know what coins are in the purse.

If we know the current state, we don't need to know the order of the coins that have been drawn to know the probability of drawing another coin.

*We can change the state variable to make a non-Markov process into a Markov process.*

In this example, if we change $X_n$ to be the number of each type of coin on the table, then we have a Markov process. *The history is encoded in the state variable itself.*

So, under the Markov assumption, we take

$p(x_t|x_{0:t},z_{1:t-1},u_{t-1})$ becomes $p(x_t|x_{t-1},u_1)$.

Something similar happens to the probability of $z_t$ given the history. This is called *conditional independence.*

Next time: *Bayes Filtering*
