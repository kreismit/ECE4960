# Lecture 16: More on Control

## Controllability Matrix

The controllability matrix $\mathbb{C}$ is derived from the impulse response of the system. A control input $u$ should affect all the outputs.

## Controllability and Reachability

If every degree of of freedom is controllable, then you can reach anywhere in your state-space domain $\mathbb{R}^n$ in a finite amount of time. (Could be a long time.)

So, controllability is equivalent to reachability.

In theory, this means we can make the system as stable as we want and we can make it react as fast as we want. In reality, the system is nonlinear, so that isn't true.

Now, all those equivalences are only true if you don't have repeated eigenvalues.

## Controllability Gramian

We want to quantify controllability beyond just a binary value.

We will use a singular value decomposition to do that.

Use the convolution equation to get a solution to the whole $\dot{x} = Ax+Bu$ equation.

The *controllability Gramian* is defined as $$w_t = \int_0^te^{At}BB^Te^{A^T\tau}dt, \quad W_t \subset \mathbb{R}^{n\times n}$$

The biggest eigenvalues of this Gramian matrix are for the most controllable directions.

This is the SVD. Do it in Matlab as follows:

```octave
C = rank(ctrb(A,b))
[U,S,V] = svd(C, 'econ'
```

Recall that the SVD of A takes the form $A = U\Sigma V^T$ where $U$ is the left singular vector, $V$ is the right singular vector, and $\Sigma$ is the diagonal matrix of singular values.

In 3D, you can draw this, as is done in the lecture slides. Given a certain amount of "energy", you'll get farther in some directions (the more controllable ones) than others; the eigenvectors define those directions.

### Stabilizability

You want all the unstable eigenvalues to be controllable. You can use the controllability Gramian to determine whether your system can be stabilized.

## Popov-Belevitch-Hautus (PBH) test

The pair $(A,b)$ (as in $\dot{x} = Ax+Bu$) is controllable if and only if

$$\text{rank} [(A-\lambda I) B] = nV \quad \lambda\subset\mathbb{C}$$

Results of this lemma (that's what a lemma is: something is true iff something else is true.)

1. The rank of $A-\lambda I$ is $n$ except in eiegenvector directions.
2. $B$ needs to have some component in each eiegenvector direction (so it can complement $A$)
3. If $B$ is a random vector (`B=randn(n,1)`), then $(A,B)$ will probably be controllable.

The only way $A$ can be rank-deficient more than once is for it to have repeated eigenvalues. If some eigenvalues are exactly equal, we will need another control input.

So, the PBH test tells you the minimum number of actuators you need.

> You end up not having to compute this value at all.

## Inverted Pendulum on a Cart


