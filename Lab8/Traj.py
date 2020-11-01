import time

class Trajectory():
    def __init__(self, loc):

        self.loc = loc
        self.robot = loc.robot
        self.mapper = loc.mapper

        # Speed Check
        # Make sure each motion is atleast more than one cell
        self.vel_cmds = {
            # [(rot1_vel_, time), (trans_vel, time), (rot2_vel, time)]
            0: [(-0.3, 0.8), (0.4, 1), (-0.1, 1.5)],
            1: [(-0.5, 1), (0.3, 1.5), (0.4, 2)],
            2: [(0, 0), (0.3, 1), (0.36, 0.3)],
            3: [(0, 0), (0.36, 1), (0.33, 2)],
            4: [(0.1, 0.5), (0.3, 1), (0.5, 2)],
            5: [(0, 0), (0.5, 1), (0, 0)],
            6: [(-0.3, 0.5), (0.42, 1), (0.3, 1)],
            7: [(-0.3, 1), (0.48, 1), (0.3, 1)],
            8: [(0.5, 2), (0, 0), (0, 0)],
            9: [(0, 0), (0.3, 1), (0.1, 0.2)],
            10: [(0, 0), (0.33, 1), (-0.3, 0.2)],
            11: [(-0.2, 0.5), (0.2, 1), (0, 0)],
            12: [(0, 0), (0.3, 1.5), (0, 0)],
            13: [(0.3, 0), (0.1, 0.5), (0.2, 0.5)],
            14: [(0, 0), (0.3, 1.5), (0, 0)],
            15: [(0.4, 3), (0, 0), (0, 0)],
            16: [(0.2, 1), (0.2, 1), (0, 0)],
            17: [(0.1, 0.5), (0.2, 1), (0, 0)],
            18: [(0, 0), (0.3, 1), (-0.1, 0.2)],
            19: [(0, 0), (0.3, 1), (-0.1, 0.2)],
            20: [(0.3, 0.2), (0.2, 0.5), (0.2, 0.5)],
            21: [(0, 0), (0.3, 1), (0, 0)],
            22: [(0.3, 1), (0.2, 2), (0.3, 1)],
            23: [(0.4, 2), (0.1, 2), (0.2, 1)],
            24: [(0.5, 1), (0.25, 1), (-0.2, 0.5)],
            25: [(0.5, 1), (0.25, 1), (-0.3, 1)],
        }

        self.total_time_steps = len(self.vel_cmds)

    def perform_motion(self, t):
        vel_cmd = self.vel_cmds[t]

        # Rot1
        i = 0
        self.robot.set_vel(0, vel_cmd[i][0])
        time.sleep(vel_cmd[i][1])
        self.robot.set_vel(0, 0)

        # Trans
        i = 1
        self.robot.set_vel(vel_cmd[i][0], 0)
        time.sleep(vel_cmd[i][1])
        self.robot.set_vel(0, 0)

        # Rot2
        i = 2
        self.robot.set_vel(0, vel_cmd[i][0])
        time.sleep(vel_cmd[i][1])
        self.robot.set_vel(0, 0)

    def execute_time_step(self, t):
        if(t == 0):
            self.robot.reset()

        # Record Odom and GT before motion
        prev_odom = self.robot.get_pose()
        prev_gt = self.robot.get_gt_pose()

        self.perform_motion(t)

        # Record Odom and GT after motion
        current_odom = self.robot.get_pose()
        current_gt = self.robot.get_gt_pose()

        return prev_odom, current_odom, prev_gt, current_gt

    def execute_entire_trajectory(self):
        self.robot.reset()

        for i in self.vel_cmds.keys():
            print("Time Step: ", i)
            self.execute_time_step(i)
    
    def execute_control(self, rot1, trans, rot2, delta_t = 1, reset=True):
        if(reset == True):
            self.robot.reset()

        # Record Odom and GT before motion
        prev_odom = self.robot.get_pose()
        prev_gt = self.robot.get_gt_pose()

        # Rot1
        self.robot.set_vel(0, rot1)
        time.sleep(delta_t)
        self.robot.set_vel(0, 0)

        # Trans
        self.robot.set_vel(trans, 0)
        time.sleep(delta_t)
        self.robot.set_vel(0, 0)

        # Rot2
        self.robot.set_vel(0, rot2)
        time.sleep(delta_t)
        self.robot.set_vel(0, 0)

        # Record Odom and GT after motion
        current_odom = self.robot.get_pose()
        current_gt = self.robot.get_gt_pose()

        return prev_odom, current_odom, prev_gt, current_gt

