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
        # Calculate the magnitude of error:

        delta_x = goal[0] - state[0]
        delta_y = goal[1] - state[1]
        print "delta x and y: ", delta_x, delta_y, "\n"

        rho = np.sqrt(np.square(delta_x) + np.square(delta_y))
        print "rho ", rho, "\n"

        #alpha = (goal[2]-state[2])
        alpha = -state[2] + np.arctan2(delta_y,delta_x)
        print "alpha ", alpha, "\n"

        # delta_x = rho * np.cos(alpha + state[2])
        # delta_y = rho * np.sin(alpha + state[2])
        # print "delta x and y: ", delta_x, delta_y, "\n"

        # beta = -np.arctan2(delta_y,delta_x)
        beta = - state[2] - alpha
        print "beta: ", beta, "\n"

        # Check if state is close enough to goal. If not:
        # Use calculated to develop an acceptable velocity and omega value.

        gain_matrix  = np.matrix([[self.kp, 0, 0], [0, self.ka, self.kb]])
        print gain_matrix
        print "\n"

        error_matrix = np.matrix([[rho], [alpha], [beta]])
        print error_matrix
        print "\n"

        vel_omega = np.matmul(gain_matrix, error_matrix)
        print "lin and ang vel: ", vel_omega, "\n"

        if vel_omega[0] > self.MAX_SPEED:
            v = self.MAX_SPEED
        else:
            v = vel_omega[0]

        if vel_omega[1] > self.MAX_OMEGA:
            omega = self.MAX_OMEGA
        else:
            omega = vel_omega[1]

        if delta_x < self.error_tol and delta_y < self.error_tol:
            done = True
        else:
            done = False

        return v, omega, done



if __name__ == '__main__':
    # Tests:

    test_start = np.array([0,0,0])
    test_goal = np.array([5,5, np.pi/4]) # 5m, 5m, 45 degrees

    diff_drive_controller = DiffDriveController(0.5, np.pi/3)  # 5 m/s, 360 Degrees
    print diff_drive_controller.compute_vel(test_start, test_goal)


