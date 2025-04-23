"""Functions for modeling ROSBot"""

# from math import cos, sin

import numpy as np
import math

from me416_utilities import stamp_difference

def model_parameters():
    """Returns two constant model parameters"""
    k = 3.0
    d = 2.0
    return k, d


def system_matrix(state_theta):
    """
    Returns a numpy array with the A(theta) matrix
    for a differential drive robot
    """
    # This is a stub. Write your code here.
    k, d = model_parameters()
    system_matrix_A = k/2 * np.array([[math.cos(state_theta), math.cos(state_theta)],
                        [math.sin(state_theta), math.sin(state_theta)], 
                        [(-1/d), (1/d)]])
    return system_matrix_A


def system_field(state_z, input_u):
    """Computes the field at a given state for the dynamical model"""
    # This is a stub. Write your code here.
    # return dot_state_z


def euler_step(state_z, input_u, step_size):
    """Integrates the dynamical model for one time step using Euler's method"""
    state_z_next = state_z + step_size * (system_matrix(state_z[2]) @ input_u)
    assert state_z_next.shape == state_z.shape
    return state_z_next

def twist_to_speeds(speed_linear, speed_angular):

    """

    Returns normalized speeds for the left and right motors.


    Inputs: speed_linear:float, speed_angular:float

    Outputs: speed_left:float, speed_right:float

    """

    # If speed_angular is close to zero, assumes straight line motion

    k, d = model_parameters()

    speed_left = (1/k) * (speed_linear - (speed_angular * d))

    speed_right = (1/k) * (speed_linear + (speed_angular * d))

    speed_left = max(min(speed_left, 1.0), -1.0)

    speed_right = max(min(speed_right, 1.0), -1.0)

    return speed_left, speed_right


class KeysToVelocities():
    '''
    Class to translate cumulative key strokes to speed commands
    '''
    def __init__(self):
        '''
        Intialize speed attributes
        '''
        # initialize attributes here
        self.speed_linear = 0.0
        self.speed_angular = 0.0
        self.speed_delta = 0.2

    def update_speeds(self, key):
        '''
        Update speeds based on keyboard command
        '''
        key = key.lower()
        # Update speeds based on keyboard command
        text_description = "No Command"
        if key == 'w':
            #Check bounds
            self.speed_linear += self.speed_delta
            self.speed_linear = min(self.speed_linear, 1.0)
            text_description = "Increase linear speed"
        elif key == 's':
            #Check bounds
            self.speed_linear -= self.speed_delta
            self.speed_linear = max(self.speed_linear, -1.0)
            text_description = "Decrease linear speed"
        elif key == 'a':
            #Check bounds
            self.speed_angular += self.speed_delta
            self.speed_angular = min(self.speed_angular, 1.0)
            text_description = "Increase angular speed"
        elif key == 'd':
            #Check bounds
            self.speed_angular -= self.speed_delta
            self.speed_angular = max(self.speed_angular, -1.0)
            text_description = "Decrease angular speed"
        elif key == 'z':
            # Set speed and message
            self.speed_linear = 0.0
            text_description = "Set linear speed to 0"
        elif key == 'c':
            # Set speed and message
            self.speed_angular = 0.0
            text_description = "Set angular speed to 0"
        elif key == 'x':
            # Set speed and message
            self.speed_linear = 0.0
            self.speed_angular = 0.0
            text_description = "Set linear and angular speed to 0"
        else:
            text_description = "Invalid command"
        return self.speed_linear, self.speed_angular, text_description

class StampedMsgRegister():
    '''
    Will get the time difference two different messages 
    '''
    def __init__(self):
        '''
        Constructor for StampedMsgRegister
        INPUTS: 
            None
        OUTPUTS: 
            None
        Initalize msg_previous to None
        '''
        self.msg_previous = None
    def replace_and_compute_delay(self, msg):
        '''
        Given a stamped message, computes delay between time stamp
        and value in time_previous
        INPUTS: 
            msg : message
        OUTPUTS:
            time_delay : float
            temp_msg : msg
        '''
        time_delay = None
        if self.msg_previous is not None:
            time_delay = stamp_difference(msg.header.stamp, self.msg_previous.header.stamp)
        temp_msg = self.msg_previous
        self.msg_previous = msg
        return time_delay, temp_msg
    def previous_stamp(self):
        '''
        Returns time stamp of previous msg
        INPUTS:
            None
        OUTPUTS:
            msg_previous.header.stamp: rclpy.time.Time
        '''
        if self.msg_previous is None:
            return None
        return self.msg_previous.header.stamp