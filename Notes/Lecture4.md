# Sensor Fusion

## What is it?

Using multiple sensors together (OR sensor(s) and input from a model) to get better data than you would get from any one individual input.

"Better"=

### Reducing noise, uncertainty, or deviation

* Cheap accelerometer: use two to hopefully average out the noise. The noise is *uncorrelated* since it's caused by stuff internal to the sensor.
* Magnetometer: Very noisy - but the noise of any two sensors are *correlated* since it's caused by external sources. Use an accelerometer and a gyro since they have uncorrelated noise.
* Low-pass filter: you know (or expect) the real data won't change suddenly, so you slow the allowed change.

### Increasing data reliability

* Suppose you have just one accelerometer. Something (or someone) hits it and it breaks. No more data.
* If you have many accelerometers, you still have data (just less of it.)
* Example: pitot tubes at the front of an airplane.
* Example: someone walks in front of a robot's distance sensor; but we have a model.

### Measure states you couldn't otherwise measure

* Binocular vision: ambiguous distance unless you can triangulate

### Increase coverage area

* Each sensor has a limited range.
* Position the sensors so that the ranges complement each other.


# IMU

This will be a more interactive lecture.

IMU's are used in virtually everything - from game controllers to smart watches to drones to airplanes to submarines to satellites!

Gives orientation, acceleration, and gravity data.

Includes three axes of each:

* Accelerometer
	* Linear acceleration, $ a = \dot{v}$
	* Units: m/s²
* Gyroscope
	* Angular velocity, $\dot{\omega} = \frac{\Delta \theta}{\Delta t} $
	* Units: °/s
* Magnetometer
	* Magnetic field strength
	* Units: μT or Gauss

### Demo

### How the accelerometer works

When the device accelerates, the mass on a spring moves and changes the capacitance of a capacitor.

### Determining roll, pitch, ~~yaw~~ from accelerometer data

In the $x-z$ plane, where $z$ is vertical:

* $a_x = g\sin \theta$
* $a_z = g\cos \theta$
* $\theta = \tan^{-1}(a_x/a_z)$

Similar calculation with $y$; you can take $\sqrt{x^2+y^2}$.

*Remember, use* `atan2` *when coding!*

These outputs are quite accurate, but also rather noisy. If we know how fast our thing can move, we can replicate a LPF in software using a *complementary filter*:
$\theta_{LPF}[n] = \alpha\theta + (1-\alpha)\theta_{LPF}[n-1]$
Yes, you keep the last input and compare the new with the old based on how much we trust the input. For example, if $\alpha=0.5$ then you are averaging the new with the old.
$\alpha=0.2$ is a nice exponential lag filter.

### Dead reckoning with the IMU

This could work really well, except for three problems:
* The IMU has some offset to begin with. You can subtract that off with proper calibration, but...
* There's gravity. Unless you're perfectly level when you start out, and you never tilt, you can never really tell whether the acceleration is due to movement or due to tilt. You could calibrate that away too...
* But there's noise. Noise, double-integrated, cannot be averaged away.

## Gyroscope

### How it Works

Uses the Coriolis effect. The mass-on-spring thing vibrates; it moves sideways (or two vibrate differently, and we use differential measurements) and the variable capacitor senses this sideways movement as proportional to the rotation rate.

### Measuring Angles & Demo

Integrate over time to get a much less noisy reading of angle. It drifts, however. So, we want to somehow fuse it with the accelerometer.

We trust the gyroscope in the short range, since it's less noisy; we trust the acclerometer in the long range since gravity doesn't drift.

So we use the complementary filter equation with the integration of the gyroscope and with the gyroscope:

$$ (1-\alpha) \dot{\theta}_g dt + \alpha \theta_a $$

The result is quite good!

## Magnetometer

Yes, there are many sources of error (anything metal or which has a current flowing through it) but, given a magnetic field which is constant in an area, you can use it to zero your gyroscope since it doesn't drift over time.

Calculate the angle from the magnetometer as `atan2(y_m,x_m)`.

Fuse this with the gyroscope yaw angle to get a more reliable angle measurement.

# Lab 3: Characterization

Do more than just play with it. Do more than just measure a number. Explain how you measured each thing and why your measurement is trustworthy and repeatable.

Feel free to (you're encouraged to) work in teams.

Deliverable: Github page *and 3-5 min presentation on 9/29*.