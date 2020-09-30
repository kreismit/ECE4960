# Lecture 6 - PID cont'd & Data Types

# More on PID's

Check out the Jupyter notebooks...

## More properties of PID's

* Integrator wind-up: if you have too much integral gain, your system gets too "heavy" and you get ringing.
* Low-pass filter: if you need to use derivative in a real system with significant noise, you can use a low-pass filter to keep the derivative from amplifying the noise.
* Derivative kick: the error changes suddenly when the setpoint changes suddenly unless you change the algorithm to use the derivative of the error only, not the entire error measurement.

## Heuristic methods

* Start with zero $k_I$ and $k_D$
* Tune $k_P$ up until you get oscillation; decrease by 2x-4x
* Either tune $k_I$ or $k_D$ next
* Maximize $k_P$ again

## Discrete PID control

You need to control the system at least 10x faster than the system can respond (Nyquist limit is 2x).

## Cascaded Control Loops

It can be easier to deal with if you have a control loop controlling another. You can isolate problems more easily, and you can get more linear behavior.

*The inner loop must run 5-10x faster than the outer loop* to avoid problems.

# Data Types

Just that a processor is x bits doesn't mean the numbers will be the same number of bits!

ARM processors use 32-bit numbers. If you need to do math with larger-size variables, the processor splits it up and carries over (like we do on paper.)

When you do an operation on numbers with different types, the processor allocates memory for the largest one. Be careful about what number is cast to what type.

You can use the C function size() to find out how many bytes each type has. Also see the slides.

The Artemis has an Arithmetic Logic Unit (ALU) as all processors do. It also has a floating-point unit which significantly speeds up floating-point calculations as compared to similar boards like the Arduino.