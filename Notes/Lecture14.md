# Lecture 14: Linear(ized) Systems

Back to *fast* robots now that we're done studying quasistatic robots.

Basic linear model: $$\dot{x} = Ax + Bu$$

But, what happens when we have a nonlinear system like that describing a fast robot?

## Review of linear systems

Equation: $$\dot{x} = Ax$$ has a typical solution $$x(t) = e^{At}x(0)$$
(Of course, the solution to the complete problem is the convolution equation, etc. Not yet.)

That gets complicated. So we use a linear transform of $x$: $z = Tx \rightarrow x = T^{-1}z$.

Substituting that into the original equation ends up giving the result

$$\dot{z} = TAT^{-1}$$

And when we find the general solution to that, *it's the same.* Picking T right makes that matrix exponential way easier to compute. That's where the eigenstuff comes in.

## Eigenvectors and Eigenvalues

$\xi$ is an eigenvector of $A$, so $A\xi = \lambda\xi$, where $\lambda$ is an eigenvalue of $A$.

So, when A multiplies an eigenvector, its effect is the same as multiplying a scalar by the vector. The eigenvalue is that scalar.

Let $T$ (as we described before) be the matrix of eigenvectors: $T = [\xi_1 \xi_2 ... \xi_n]$

Let $D$ be the diagonalized matrix of eigenvalues:
$$D = \begin{bmatrix} \lambda_1 &  0 & ... & 0\\ 
0 & \lambda_2 & ... & 0\\
0 & 0 & ... & \lambda_n \end{bmatrix} $$

Then, we can decompose $A$ as $TDT^{-1}$ ($AT = TD$).

Of course, this property makes the matrix exponential much easier to compute...

Take the state, $x = \begin{bmatrix} x_1 \\ ... \\ x_n \end{bmatrix}$.
Do a linear transform from $x$ to $z$. This is not the same thing as transforming to state-space.
$$ x = Tz$$
$$ \dot{x} = T\dot{z} = Ax$$
$$ T\dot{z} = ATz$$
$$\dot{z} = T^{-1}ATz = Dz \quad !$$

We have now decoupled our system in $z$-space.

Easy to do on the computer: `[T,D] = eig(a)` in Matlab or use [linalg.eig in Numpy.](https://numpy.org/doc/stable/reference/generated/numpy.linalg.eig.html)

Now that we've decoupled this, the matrix exponential is a bunch of individual *scalar* calculations:
$$z_i(t) = e^{\lambda_it}z_i(0) \text{ for all } i$$

And then Prof. Petersen derived the answer to MAE 5780 Homework #2. You can find the matrix exponential easily with D, left-multiply the entire thing by $T$, and right-multiply by $T^{-1}$.

So, $e^{At} = Te^{Dt}T^{-1}$.

## Converting to $z$-coordinates

$z(0)$ is just $T^{-1}x(0)$. So, you map the initial condition into $z$ coordinates, find the solution in $z$ space, and then convert back to $x$ space.

$$x(t) = Te^{-Dt}T^{-1}x(0)$$

## Stability and Eigenvalues

If the real part of *any* of the eigenvalues of the matrix $A$ (any of the numbers in $D$) are positive, then the solution is unstable and blows up. If all real parts are negative, then the solution is stable. If all real parts are zero, it's a boundary case.

## Discrete Time

Let $\tilde{A} = e^{A \Delta t}$, where $\Delta t$ is the time step of the discrete system. Then, we can make this a difference equation instead of a (continuous) differential equation: each step, multiply by $\tilde{A}$. With our handy-dandy transformation, that's as simple as multiplying by eigenvalues.

Then, the system is stable when the eigenvalues are within a circle (with radius 1) rather than when the eigenvalues are in the right half-plane.