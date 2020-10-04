# Lecture 7 - Probability!

> This should hopefully be a review for most of you... It will help you get ready for the next couple of lectures where Vivek takes over and talks about localization and mapping.

## Probability << Noise

Accelerometer readings (gyro too?)

Take mean reading; and report standard deviation as well.

Dealing with noise...

* Low-pass filter >> rolling average
* Look at the sample as a probability distribution

### Normal Distributions

Described by μ and σ (mean and std. dev.)

Symmetric

The area under the curve always sums to 1.

### Bayesian Inference

Guessing in the style of Bayes. (Presbyterian minister who wrote two books: one on theology and one on probability.)

How do we determine whether a robot's failure to move is due to hardware or software?

* About 96% of broken robots don't move.
* About 40% of working robots don't move.
* Use Bayesian inference to get the probability that the non-moving robot doesn't move.
	* Say, total 100 bots.
	* 48 broken still.
	* 2 broken moving.
	* 20 working still.
	* 30 working moving.
	* Given the robot is not moving, the chance that it is broken is 48/68 = 71%.

```
P(still | broken) = # broken & still / (# broken)
```

where | (pipe) means "given".

#### Bayes' Rule

$$ P(A|B) = \frac{P(B|A)P(A)}{P(B)} $$

Some terms: $P(A|B)$ is the *conditional probability posterior.* $P(B|A)$ is the *likelihood.* $P(A)$ is the *prior* and $P(B)$ is the *marginal likelihood.*

*Joint probability* of A and B can be written $P(A, B)$ or $P(A \cap B)$ or $P(A \& B)$.

### Dependent and Independent

## Another Example

100 people go to the doctor after getting tick bites. They are getting tested for Lyme's.

* 16 test positive.
* 84 test negative.
* 40% of Lyme's patients test positive (too early.)
* 10% of Lyme-free patients test positive.
* The probability that you have the disease, given you have...
	* A positive test: 50%
	* A negative test: 14%

Some terms:

* Probability of having a positive test when you have a disease: *sensitivity*
* Probability of having a negative test when you don't have a disease: *specificity*

## Frequentist Statistics vs. Probability Distributions >> Inference

* Mean
* Std. Dev.
* Variance
* Standard error: $\sigma/\sqrt{N}$

We're measuring the speed of the robot. Use Bayes' Theorem to improve on each observation.

Assume that $P(S|M) = P(M|S)$ (conditional probabilities of speed and measurement are symmetric.)

Start with no prior assumption: uniform distribution. Guess an actual speed; come up with the probability that we will get those measurements, given that the real speed is that. Keep guessing to increase the probability.

Then, use a prior assumption of a normal distribution with the mean and std. dev. we found.

You get a much sharper-peaked distribution after iteratively using Bayes' theorem. That's called the Maximum A Priori (MAP) estimate.

## Takeaway

Always believe the impossible, just a little bit!