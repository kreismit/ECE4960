# Ideas

1. Wall following.
    * Method
        * Prox sensor is on right-hand side, and ToF sensor faces forward. (Might want to switch!)
        * Start next to a wall.
        * Prox sensor detects changes in the wall - e.g. getting closer or farther away.
        * Maintains a constant distance from the wall.
        * If wall suddenly vanishes (right angle) swing-turn right until the bot sees it again.
        * If wall in front, turn left until the new wall is on the right-hand side.
    * Pros
        * Sidesteps noisy odometry problem
        * Doesn't require map or path planner
        * Can be open-loop
    * Cons
        * Suboptimal
        * Prox sensor will give different readings with different surfaces, so might get confused
        * Might have to drive slowly to do wall following right
        * **The robot still needs to know when it's reached the goal.**
            * Can determine distance traveled using ToF in front.
            * Using distance traveled and direction from IMU, do accurate odometry.
            * ***Robot has to know the map anyway!***
            * Oh, by the way, some walls will be out of range of the TOF, so this solution might not work.
2. Bayes filter with map and modified odometry.
    * Method
        * Uses TOF and IMU rotation.
        * Perform one observation loop and localize (0th Bayes filter step.)
        * Plan a path that always faces a wall at a right angle.
        * Do odometry using PID to drive straight and TOF to tell distance from wall.
        * 1st Bayes filter step. Where am I? I have real odometry data now!
        * Do it again for the next step of the path. (Maximum of 2-3 steps given shape of room.)
    * Pros
        * Eliminates odometry problem
        * Uses pre-existing Bayes Filter setup
    * Cons
        * Requires weird path planner
        * Requires multiple observation steps: *slow!*
3. Bayes filter with really good update step.
    * Method
        * Run 0th step
        * Run path planner that only faces walls *or* traditional path planner with really good odometry system
        * Perform all steps to get to goal
        * Localize again and see if reached goal
    * Pros
        * Relatively fast *if* reach goal the first time
        * Pros of using ToF to do odometry
    * Cons
        * Has to stop and turn in place (still slow)
        * If localization is performed incorrectly or robot doesn't reach goal the first time, quite slow
4. Bug algorithm, using magnetometer to tell when reach goal
    * Method
        * Place magnet on floor in location of goal
        * Follow walls, etc. until detect location of strongest field
    * Pros
        * Pros of wall following
    * Cons
        * Cons of wall following
        * Influenced by other magnetic fields (e.g. Earth)
        * Requires magnet (pretty cheap though)
5. Bug algorithm, using buzzer ringing continuously
    * Method
        * Same as magnet idea, except uses buzzer over goal
        * Bug algorithm until maximum loudness
    * Pros
        * Only need to know amplitude of buzzer
        * Pros of wall following / bug algorithm
    * Cons
        * Heavily influenced by other noises
        * Mic isn't directionally sensitive
        * How do you know you're at "max" loudness? Predefined? What if it changes based on echoes, etc.?
6. Just make localization and odometry really good. Use velocity model so path planning can be fast and run once.
    * Pros
        * Fast
        * Well-researched ideas
    * Cons
        * Relies on perfect localization - what if you are wrong and crash?
        * Don't know how to improve IMU data - velocity model probably won't work (or will take days to make work)
