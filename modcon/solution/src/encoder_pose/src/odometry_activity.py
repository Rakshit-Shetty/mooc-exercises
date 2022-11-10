#!/usr/bin/env python
# coding: utf-8

# In[16]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

#TODO: write a correct function

def DeltaPhi(encoder_msg, prev_ticks):
    """
        Args:
            encoder_msg: ROS encoder message (ENUM)
            prev_ticks: Previous tick count from the encoders (int)
        Return:
            rotation_wheel: Rotation of the wheel in radians (double)
            ticks: current number of ticks (int)
    """
    
    # TODO: these are random values, you have to implement your own solution in here
    #Read ticks from encoder_msg
    ticks = encoder_msg.data    
    
    #Difference with the previous call
    delta_ticks = ticks - prev_ticks
    
    #Wheel Rotation
    N_tot = encoder_msg.resolution
    
    alpha = 2 * np.pi/N_tot
    
    delta_phi = alpha * delta_ticks

    return delta_phi, ticks

# In[17]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your odometry function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

# TODO: write the odometry function

import numpy as np 

def poseEstimation( R, # radius of wheel (assumed identical) - this is fixed in simulation, and will be imported from your saved calibration for the physical robot
                    baseline_wheel2wheel, # distance from wheel to wheel; 2L of the theory
                    x_prev, # previous x estimate - assume given
                    y_prev, # previous y estimate - assume given
                    theta_prev, # previous orientation estimate - assume given
                    delta_phi_left, # left wheel rotation (rad)
                    delta_phi_right): # right wheel rotation (rad)
    
    """
        Calculate the current Duckiebot pose using the dead-reckoning approach.

        Returns x,y,theta current estimates:
            x_curr, y_curr, theta_curr (:double: values)
    """
    
    # TODO: these are random values, you have to implement your own solution in here
    
    #Distance travelled by each wheel
    
    d_left = R * delta_phi_left
    d_right = R * delta_phi_right
    
    #Distance covered by duckitbot in robot frame
    d_A = (d_left + d_right)/2
    
    #Rotation of the robot
    Dtheta = (d_right - d_left)/baseline_wheel2wheel
    
    #Distance covered by duckiebot in world frame
    Dx = d_A * np.cos(theta_prev)
    Dy = d_A * np.sin(theta_prev)
    
    #Current Pose
    x_curr = x_prev + Dx 
    y_curr = y_prev + Dy 
    theta_curr = theta_prev + Dtheta

    return x_curr, y_curr, theta_curr
