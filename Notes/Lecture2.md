## How do we represent motion in *x* degrees of freedom?
* Example: Robot with four independent joints
* Example: RC car constrained to a plane
* Example: Wooden block floating in space

## Toruses and Spheres
* If you sweep through all the possible points of motion, you get a surface.
* If the surface is a sphere, cylinder, etc., then we have singularities.
	* If I step over the north pole...
	* If I have 4 DoF, I get two spheres on top of each other...
* If you use an implicit representation, you eliminate singularities...
	* $x^2+y^2+z^2=1$ is an example for a sphere.
	* Use two coordinates to find the third.
	* They also have problems (you have to solve an equation every time)
* If you use a rotation matrix, you can still get gymbal lock.
* Quaternions are the best. We will use them.

## Coordinate Frames
* Right-hand rule
	* First three fingers are *x*, *y*, *z*
	* Cross products and fingers curl
* An inertial frame could be
	* the frame of the world
	* the frame of space
	* we will use the three interchangeably
* Body frame
	* is NOT inertial (in our case, anyway)
* We will number our frames
	* Robots often have more than one non-inertial frame
	* *P^0* is a point *expressed* in the inertial frame
	* *P^1* is a point *expressed* a body frame
	* The superscript is always the the number the vector is expressed in.

## Homogeneous transformation matrix
* Input coordinates in the body frame
* Output coordinates in the world frame
* Includes transformation and rotation matrices
* When you transform O^1 to frame 0, it is denoted $O^0_1$.
	* The number of the frame the vector is expressed in is always in the superscript.
* To transform from one frame to another, you **multiply** the transformation matrix by the coordinate vector.
	* Rotation: Use a trigonometric rotation matrix. (2x2 for the 2-D case.)
		* Defined as $R_1^0 = \begin{bmatrix} x_0 \cdot x_1 & y_0 \cdot y_1 \\ x_0 \cdot y_1 & y_0 \cdot y_1 \end{bmatrix} $
		* When you extend this to 3-D, the definition works the same way.
		* If you do a 3-D rotation which is constrained to the $x-y$ plane, it is $R_1^0 = \begin{bmatrix} x_0 \cdot x_1 & y_0 \cdot y_1 & 0 \\ x_0 \cdot y_1 & y_0 \cdot y_1 & 0 \\ 0 & 0 & 1 \end{bmatrix} $
		* Angles in 3-D are *roll, pitch,* and *yaw*.
			* Roll = $\phi$
			* Pitch = $\rho$
			* Yaw = $\theta$
			* Some people define *z* to point downward.
	* Translation: Add the vectors.
	* Result: you get a 4&times;4 matrix
		* Rotation is in the 3&times;3 upper left corner
		* Translation, $d_x$, $d_y$, and $d_z$, is in the upper left corner
		* The bottom row is $[\ 0 \ 0 \ 0 \ 1\ ]$
* An exercise
	* Given a ToF point $\begin{bmatrix} d_m \\ 0 \\ 0 \\ 1 \end{bmatrix}$
	* In the robot's frame of reference
	* Find the point in the inertial frame

### Euler Angles
* Any rotation can be described by three successive rotations about linearly independent axes.
* Proper Euler angles *z-x-z*, *x-y-x*, ...
* Talt-Bryan angles *x-y-z*, ...
* Robots usually use the **$z-y-z$** convention or the **$x-y-z$** convention. 
	* $z-y-z$
		* Rotation around $z$ is $\phi$
		* Rotation about $y$ is $\theta$
		* Rotation about $x$ is $\psi$
	* Roll-Pitch-Yaw ($x-y-z$)
		* About $x$: $\phi$
		* About $y$: $\theta$
		* About $z$: $\psi$
	* "Literally plug and play."
	* Note that the inverse problem is *problematic*.
		* The solution to `acos` is not unique
		* Use `atan2(adj, opp)` not `atan(opp/adj)`
			* This is not consistent across platforms!