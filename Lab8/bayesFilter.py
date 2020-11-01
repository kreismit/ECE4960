# Import useful Numpy functions
from numpy import arctan2, hypot, pi, deg2rad, rad2deg, arange
# and alias several functions to prevent errors later
odom_rot_sigma = loc.odom_rot_sigma
odom_trans_sigma = loc.odom_trans_sigma
sensor_sigma = loc.sensor_sigma
gaussian = loc.gaussian
normalize_angle = mapper.normalize_angle
to_map = mapper.to_map
toMap = to_map
from_map = mapper.from_map
fromMap = from_map

# In world coordinates
def compute_control(cur_pose, prev_pose):
    """ Given the current and previous odometry poses, this function extracts
    the control information based on the odometry motion model.

    Args:
        cur_pose  ([Pose]): Current Pose
        prev_pose ([Pose]): Previous Pose 

    Returns:
        [delta_rot_1]: Rotation 1  (degrees)
        [delta_trans]: Translation (meters)
        [delta_rot_2]: Rotation 2  (degrees)
    Pseudocode (well, this code probably compiles):
        delta_y = cur_pose[1] - prev_pose[1]
        delta_x = cur_pose[0] - prev_pose[0]
        delta_rot_1 = atan2(delta_y, delta_x)
        delta_trans = sqrt(delta_y^2 + delta_y^2)
        delta_rot_2 = cur_pose[2] - (prev_pose[2] + delta_rot_1)
    """
    dy = cur_pose[1] - prev_pose[1] # pre-compute these since they'll be
    dx = cur_pose[0] - prev_pose[0] # used twice
    # normalize the change in angle (could be >180°)
    dTheta = normalize_angle(cur_pose[2] - prev_pose[2])
    delta_rot_1 = rad2deg(arctan2(dy, dx))
    delta_trans = hypot(dx, dy) # get magnitude of dx, dy
    delta_rot_2 = dTheta - delta_rot_1
    return delta_rot_1, delta_trans, delta_rot_2

# In world coordinates
def odom_motion_model(cur_pose, bel, u):
    """ Odometry Motion Model

    Args:
        cur_pose  ([Pose]): Current Pose (x,y,th) in meters
        bel ([ndarray 20x20x18]): Belief about position from last iteration
        u = (rot1, trans, rot2) (float, float, float): A tuple with control data
            in the format (rot1, trans, rot2) with units (degrees, meters, degrees)

    Returns:
        prob [scalar float]: Probability sum over x_{t-1} (p(x'|x, u)) at the cur_pose.
        
    Pseudocode (MODIFIED):
        x, y, th = prev_pose
        for prevX,prevY,prevTh in gridPoints: # use fromMap() to get these coords.
            # Figure out what each movement would have been to get here
            dx = x-prevX; dy = y-prevY; dth = theta-prevTh;
            dtrans = sqrt(dx^2+dy^2)
            drot1 = atan2(dy,dx)
            drot2 = dth - rot1i
            pR = gaussian(trans, dtrans, odom_trans_sigma)
            pTh1 = gaussian(rot1, drot1, odom_rot_sigma)
            pTh2 = gaussian(rot2, drot2, odom_rot_sigma)
            pXYT = pR*pTh1*pTh2 # probability we got all three right
            prob[x,y,th] = prob[x,y,th] + pXYT*bel[prevX,prevY,prevTh]
    """
    # Determine what each movement would have been to get here: 20x20x18 array
    # We don't know where we are or where we were, but we have
    # * A guess for where we are now (cur_pose).
    # * A distribution for where we were. Sum over that in this function.
    drot1 = np.empty([20,20,1])   # first rotation to travel to point x,y
    dtrans = np.empty([20,20,1])  # translation to travel to x,y after rotating
    drot2 = np.empty([20,20,18])  # second rotation to achieve angle theta
    (x,y,th) = mapper.to_map(*cur_pose) # actually the indices for x,y,th
    # Use vectorized code to calculate prevX & prevY fast (prevTh is a dummy for now)
    (prevX,prevY,prevTh) = mapper.from_map(arange(0,20),arange(0,20),1)
    dx = cur_pose[0]-prevX # change in x position (m) - array size 20
    dy = cur_pose[1]-prevY # change in y position (m) - array size 20
    # ^ Just do this once. th has to be recalculated many times because
    # from_map() can't handle multiple values of theta at once.
    for i in range(20):
        dtrans[i,:,0] = hypot(dx[i],dy)   # do all of y at once; loop through x
        drot1[i,:,0] = rad2deg(arctan2(dy,dx[i]))   # both 20x20 arrays
    for i in range(18): # loop over all angles and overwrite th
        prevTh = mapper.from_map(1,1,i)[2]  # don't care about 0 and 1
        # Normalize the change in angle (could be >180°)
        dth = normalize_angle(cur_pose[2] - prevTh)
        drot2[:,:,i] = dth - drot1[:,:,0]  # build the entire array!
    # Probability of being in this pose given the control action taken:
    # use Law of Total Probability and sum over all possible starting points.
    # This is the entire RHS of the pseudocode for the prediction step.
    prob = sum(sum(sum(
              gaussian(u[0], drot1, odom_rot_sigma)
            * gaussian(u[1], dtrans, odom_trans_sigma)
            * gaussian(u[2], drot2, odom_rot_sigma) 
            * bel )))
    return prob

def prediction_step(cur_odom, prev_odom, bel):
    """ Prediction step of the Bayes Filter.
    Update the probabilities in loc.bel_bar based on loc.bel (bel)
    from the previous time step and the odometry motion model.

    Args:
        cur_odom  ([Pose]): Current pose, estimated from odometry
        prev_odom ([Pose]): Previous pose, estimated from odometry
        loc.bel [ndarray 20x20x18]: Previous location probability density
        
    Returns:
        loc.bel_bar [ndarray 20x20x18]: Updated location probability density.
        This will be the prior belief for the update step.
        
    Pseudocode (FIXED):
        for x, y, theta in configuration space:
            loc.bel_bar[x,y,theta] = odom_motion_model((x,y,th),bel,u)
            # Plugs in all possible current values (t).
            # odom_motion_model() takes care of
            # summing over all possible previous values (t-1).
    """
    u = compute_control(cur_odom,prev_odom) # What control action did we take?
    bel_bar = np.zeros([20,20,18]) # Initialize prior belief with correct dims
    for k in range(18):     # loop over all possible current poses,
        for j in range(20): # in configuration space
            for i in range(20):
                pose = mapper.from_map(i,j,k) # from config space to world
                # odom_motion_model sums probability of getting here from
                # all possible previous poses
                bel_bar[i,j,k] = odom_motion_model(pose,bel,u)
                # so bel_bar is the probability of being at indices i,j,k
    # Below: just for debugging
#     (i,j,k) = (10,10,9)
#     print("bel_bar:")
#     print(bel_bar[i,j,k])
#     print(bel_bar.shape)
    return bel_bar

def sensor_model(obs):
    """ This is the equivalent of p(z|x). Checks all possible poses x_t.
    Args:
        obs [ndarray]: A 1D array consisting of the measurements made in rotation loop
                            1D array of size 18 (=loc.OBS_PER_CELL)

    Returns:
        probArray [ndarray]: Returns a 20x20x18 array corresponding to
                             p(z_t|x_t) for all values of x_t.
    Pseudocode:
        (x,y,theta) = cur_pose
        for i in range(18):
            d = getViews(x,y,theta)
            probArray[i] = gaussian(obs[i], d, sensor_sigma)
    """
    probArray = gaussian(obs[0], mapper.obs_views[:,:,:,0],sensor_sigma)
    for i in range(1,18): # Probability of getting ALL the sensor readings
                        # p(z1 & z2 & ...) = p(z1)p(z2)...
        probArray = probArray*gaussian(obs[i], mapper.obs_views[:,:,:,i],sensor_sigma)
        # this is elementwise multiplication
    # for debugging
#     eta = sum(sum(sum(probArray)))
#     print("In sensor model:")
#     if np.isfinite(eta):
#         print("Eta is finite and equals {}".format(eta))
#     elif abs(eta) < 1e-10:
#         print("Eta is zero!")
#     else:
#         print("Eta is infinite!")
#     print("probArray:")
#     print(probArray.shape)
    probArray = probArray / sum(sum(sum(probArray)))  # normalized probability
    # need to sum over all three indices to get total
    return probArray

# In configuration space
def update_step(bel_bar, obs):
    """ Update step of the Bayes Filter.
    Update the probabilities in loc.bel based on loc.bel_bar and the sensor model.
    Args:
        loc.bel_bar [ndarray 20x20x18]: belief after prediction step
        obs [ndarray]: A 1D array consisting of the measurements made in rotation loop
    Returns:
        loc.bel [ndarray 20x20x18]: belief after update step
    Pseudocode:
        loc.bel = sensorModel(obs)*loc.bel_bar
        eta = 1/sum(loc.bel)    # normalization constant
        loc.bel = loc.bel*eta
    """
    bel = sensor_model(obs)*bel_bar
    #bel = bel_bar  # for debugging: use only the motion model
    #bel = sensor_model(obs) # for debugging: use only the sensor model
    # for debugging
#     eta = sum(sum(sum(bel)))
#     print("In update step:")
#     if np.isfinite(eta):
#         print("Eta is finite and equals {}".format(eta))
#     elif abs(eta) < 1e-10:
#         print("Eta is zero!")
#     else:
#         print("Eta is infinite!")
#     print("bel:")
#     print(bel.shape)
    bel = bel/sum(sum(sum(bel))) # normalized
    # need to sum over all three indices to get total
    return bel
