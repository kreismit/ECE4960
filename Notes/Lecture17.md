# Lecture 17 &ndash; Inverted Pendulum and LQR

## PID on inverted pendulum

Nice review of MAE 5730 (with different methods) and the state-space portion of MAE 5780.

We can push the eigenvalues really far into the LHP to get really good performance, but then our linear model breaks down and the system is unstable.

We can play it safe with regards to gains and get something which is really slow and bad.

If we don't have a specific phase margin requirement, we can use a technique called **LQR** (Linear Quadratic Regulator) to find the "best" eigenvalues.

## Linear Quadratic Regulator

In Matlab: `K = lqr(A,B,Q,R)`

A and B are the $A$ and $B$ of the linear system.

$Q$ and $R$ are two cost matrices which tell the algorithm how annoying it is to be far away from the right eigenvalues.

$$\int_0^\infty x^TQx+u^TRu dt$$

*Ricatti equation*

Some special algorithms to get $Q$ and $R$. This method can get computationally expensive for complicated systems, but for our 2-D inverted pendulum problem, it's pretty quick.

We can adjust the $Q$ and $R$ matrices to

* $Q$: penalize spending too much control energy (corresponding to physical limits)
* $R$: penalize spending too much time (corresponding to performance requirements)

Typically people try a value (say 1) and then up the value by a factor of ten until it looks reasonable.