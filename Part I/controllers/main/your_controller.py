# Fill in the respective functions to implement the controller

# Import libraries
import numpy as np
from base_controller import BaseController
from scipy import signal, linalg
from util import *

# CustomController class (inherits from BaseController)
class CustomController(BaseController):

    def __init__(self, trajectory):

        super().__init__(trajectory)

        # Define constants
        # These can be ignored in P1
        self.lr = 1.39
        self.lf = 1.55
        self.Ca = 20000
        self.Iz = 25854
        self.m = 1888.6
        self.g = 9.81
        self.e_integral_x=0
        self.e_previous_x=0
        self.e_integral_y=0
        self.e_previous_y=0
        # Add additional member variables according to your need here.

    def update(self, timestep):

        trajectory = self.trajectory

        lr = self.lr
        lf = self.lf
        Ca = self.Ca
        Iz = self.Iz
        m = self.m
        g = self.g
        e_integral_x= self.e_integral_x
        e_previous_x=self.e_previous_x
        e_integral_y= self.e_integral_y
        e_previous_y=self.e_previous_y
        
        # Fetch the states from the BaseController method
        delT, X, Y, xdot, ydot, psi, psidot = super().getStates(timestep)

        # Design your controllers in the spaces below. 
        # Remember, your controllers will need to use the states
        # to calculate control inputs (F, delta). 

        v_desired=20
        dist, index = closestNode(X, Y, trajectory)
        future_index=index+60
        # if ( future_index >= trajectory.shape[0]):
        #     future_index = trajectory.shape[0]
        if ( index >= trajectory.shape[0]-60):
            future_index = trajectory.shape[0]-1
        # closest_point=trajectory[index]
        future_point=trajectory[future_index]


        # F=10000
        # delta=0
        # ---------------|Lateral Controller|-------------------------

        Kp_y=9
        Ki_y=0
        Kd_y=0
        
        psi_desired = np.arctan2(future_point[1] - Y, future_point[0] - X)
        e_y=wrapToPi(psi_desired-psi)
        e_integral_y=e_integral_y+e_y
        delta= Kp_y*e_y + Ki_y*e_integral_y + Kd_y*(e_y-e_previous_y)/delT
        if np.absolute(delta)>np.pi/6:
            if delta<0:
                delta=-np.pi/6
            else:
                delta=np.pi/6

        e_previous_y=e_y
        # ---------------|Longitudinal Controller|-------------------------
        """        Please design your longitudinal controller below.
        """
        Kp_x=50
        Ki_x=0
        Kd_x=0
        
        e_x= v_desired-xdot
        e_integral_x=e_integral_x+e_x
        F = Kp_x*e_x + Ki_x*e_integral_x + Kd_x*(e_x-e_previous_x)/delT
        if F<0:
            F=0
        if F>15736:
            F=15736
        e_previous_x=e_x
        
        # Return all states and calculated control inputs (F, delta)
        return X, Y, xdot, ydot, psi, psidot, F, delta
