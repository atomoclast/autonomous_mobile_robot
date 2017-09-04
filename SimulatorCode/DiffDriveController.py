#!/usr/bin/python

import numpy as np


class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega):
        self.kp=1  # Must be positive
        self.ka=2  # Must be positive with ka - kp > 0
        self.kb=-1 # Must be negative
        self.error_tol = 0.05
        self.MAX_SPEED = max_speed
        self.MAX_OMEGA = max_omega

        # Characteristic Polynomial: 
        # (lambda + kp)(lambda^2 + (ka - kp)*lambda - kp*kb) = 0


        
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should 
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """

        KP = self.kp
        KA = self.ka
        KB = self.kb
        delx = goal[0] - state[0]
        dely = goal[1] - state[1]
        theta = state[2]
        tworoots = np.roots([1, KA - KP, -KP * KB])
        w = [-KP, tworoots[0], tworoots[1]]
        # if w[0]<0 and w[1]<0 and w[2]<0:
        p = np.sqrt((delx ** 2) + (dely ** 2))
        print(p)
        alpha = -theta + np.arctan2(dely, delx)
        beta = -theta - alpha

        v = p * KP
        if np.all(v > self.MAX_SPEED):
            v = self.MAX_SPEED

        omega = (alpha * KA) + (beta * KB)
        if np.all(omega > self.MAX_OMEGA):
            omega = self.MAX_OMEGA

        if np.all(p < 0.15):
            done = True
        else:
            done = False

        vw = (v, omega, done)
        print("vw: ", vw)
        return vw


if __name__ == '__main__':
    # Tests:

    test_start = np.array([0,0,0])
    test_goal = np.array([5,5, np.pi/4]) # 5m, 5m, 45 degrees

    diff_drive_controller = DiffDriveController(0.5, np.pi/3)  # 5 m/s, 360 Degrees
    print diff_drive_controller.compute_vel(test_start, test_goal)


