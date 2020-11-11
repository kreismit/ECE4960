# Lecture 15: Linearization and Controllability

## How to Linearize a Nonlinear System

1. Find some fixed (equilibrium) points.
2. Find the Jacobian of the system at that point.

Now we have a linearized system $\dot{x} = Ax$, where $A$ is the Jacobian matrix.

A reminder: the formula for the Jacobian is:
$$ \begin{bmatrix} \partial f_1/\partial x_1 & \partial f_1 / \partial x_2 \\
\partial f_2/\partial x_1 & \partial f_2 / \partial x_2 \end{bmatrix}$$

## Is a System (Matrix) Controllable?

We have a linear system

$$\dot{x} = Ax + Bu$$

It has a feedback with a linear control law $-Kx$. This could describe a lead-lag controller, a PID controller, anything. This controller is optimal if the correct $K$ is chosen... if the system is controllable.

(If you can control based on all the values in the state vector $x$, that's called *full-state feedback.*)

*A system is controllable if you can steer your state anywhere you want in $\mathbb{R}$.*

Usually, we don't get to choose $A$ or $B$. We only get to choose $K$.

> Kind of the holy grail of control - even if I have really complex dynamics, as long as the degrees of freedom are tightly coupled, I can write a control law which will control it with a minimal number of actuators.

However, if one of the degrees of freedom isn't controllable, then changing $u$ won't change it.

### Calculating the Controllability Matrix

$$\mathbb{C} = [B \quad AB \quad A^2B \quad ... \quad A^{n-1}B]$$

If the rank of $\mathbb{C}$ is $n$ (full column rank) then the system is controllable.

In Matlab:

    ctrb(A,B)

In Python (import the [`control` library](https://python-control.readthedocs.io/en/0.8.1/generated/control.ctrb.html)):

    control.ctrb(A,B)

