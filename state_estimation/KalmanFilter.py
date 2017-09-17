#!/usr/bin/python
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import patches
#import pylab
import time
import math

class KalmanFilter:
    """
    Class to keep track of the estimate of the robots current state using the
    Kalman Filter
    """
    def __init__(self, markers):
        """
        Initialize all necessary components for Kalman Filter, using the
        markers (AprilTags) as the map
        Input: 
        markers - an N by 4 array loaded from the parameters, with each element
            consisting of (x,y,theta,id) where x,y gives the 2D position of a
            marker/AprilTag, theta gives its orientation, and id gives its
            unique id to identify which one you are seeing at any given
            moment
        """
        # x_r = markers[0]
        # y_r = markers[1]
        # theta_r = markers[2]
        # H_robot = np.array([[np.cos(theta_r), -np.sin(theta_r), x_r],
        #                    [np.sin(theta_r), np.cos(theta_r), y_r],
        #                    [0, 0, 1]])
        #
        # H_robot_2_world_t = np.dot(np.linalg.inv(H_robot.transpose()), H_world)
        # H_robot_2_world = H_robot_2_world_t.transpose()
        # print "H_robot_2_world: ", H_robot_2_world

        self.markers = markers
        self.last_time = time.time() # Used to keep track of time between measurements
        self.Q_t = np.eye(2)   # Covar of Process Noise
        self.R_t = np.eye(3)   # Covar of Sensor Variance accel_x, accel_y, omega
        self.x = self.markers[0]  # BotX_world
        self.y = self.markers[1]  # BotY_world
        #BotTheta_world
        self.theta = np.arctan2(np.sin(self.markers[2]), np.cos(self.markers[2]))
        self.u_t_hat = None
        self.u_t_next = None
        self.u_t_last = None
        self.E_t_hat = None
        self.E_t_next = None
        self.E_t_last = None
        self.K_t_next = None
        self.K_t_last = None
    def prediction(self, v, imu_meas):
        """
        Performs the prediction step on the state x_t and covariance P_t
        Inputs:
        v - a number representing in m/s the commanded speed of the robot
        imu_meas - a 5 by 1 numpy array consisting of the values
            (acc_x,acc_y,acc_z,omega,time), with the fourth of the values giving
            the gyroscope measurement for angular velocity (which you should
            use as ground truth) and time giving the current timestamp. Ignore
            the first three values (they are for the linear acceleration which
            we don't use)
        Outputs: a tuple with two elements
        predicted_state - a 3 by 1 numpy array of the predation of the state
        predicted_covariance - a 3 by 3 numpy array of the predation of the
            covariance
        """
        t_now =  time.time()

        dt = time_now - self.last_time

        # Jacobian - df/dx
        Jx = np.eye(3) + dt * np.array([[0, 0, -v * np.sin(self.theta)], [0, 0, v * np.cos(self.theta)], [0, 0, 0]])

        # Jacobian - df/dn
        Jn = dt * np.array([[np.cos(self.theta), 0], [np.sin(self.theta), 0], [0, 1]])

        # Jacobian - dh/dx
        Jhx = np.eye(3)

        # Estimating the state
        self.u_t_hat = self.u_t_last + dt * np.array(
            [[(v + nv) * np.cos(self.theta)], [(v + nv) * np.sin(self.theta)], [omega + nw]])

        # estimating the Process Covariance

        self.E_t_hat = Jx * self.E_t_last * Jx.transpose() + np.dot(np.dot(Jn, Q), Jn.transpose())

        # Update Kalman Gain
        K_t = np.dot(np.linalg.inv((Jhx * P_t_estimate * Jhx.transpose()) + R).transpose(),
                     (self.E_t_next * Jhx.transpose()).transpose())
        K = K_t.transpose()

        self.last_time = time.time()

        pass

    def update(self, z_t):
        """
        Performs the update step on the state x_t and covariance P_t
        Inputs:
        z_t - an array of length N with elements that are 4 by 1 numpy arrays.
            Each element has the same form as the markers, (x,y,theta,id), with
            x,y gives the 2D position of the measurement with respect to the
            robot, theta the orientation of the marker with respect to the
            robot, and the unique id of the marker, which you can find the
            corresponding marker from your map
        Outputs:
        predicted_state - a 3 by 1 numpy array of the updated state
        predicted_covariance - a 3 by 3 numpy array of the updated covariance
        """
        self.u_t_next = self.u_t_hat + np.dot(K, (z_t - self.u_t_hat))

        self.E_t_next = P_t_estimate - K * Jhx * P_t_estimate

    def step_filter(self, v, imu_meas, z_t):
        """
        Perform step in filter, called every iteration (on robot, at 60Hz)
        Inputs:
        v, imu_meas - descriptions in prediction. Will be None value if
            values are not available
        z_t - description in update. Will be None value if measurement is not
            available
        Outputs:
        x_t - current estimate of the state
        """

        pass


if __name__ == "__main__":
    print "this is the main script."
    test_artag = np.array([1, 2, .5, 0.0])
    ekf = KalmanFilter(test_artag)
    print "done."
