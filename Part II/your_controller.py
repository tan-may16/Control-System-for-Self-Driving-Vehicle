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
        # Fetch the states from the BaseController method
        delT, X, Y, xdot, ydot, psi, psidot = super().getStates(timestep)
        
        # Design your controllers in the spaces below. 
        # Remember, your controllers will need to use the states
        # to calculate control inputs (F, delta). 

        # ---------------|Lateral Controller|-------------------------
        """
        Please design your lateral controller below.
        """

        dist, index = closestNode(X, Y, trajectory)
        future_index=index+100
        # if ( future_index >= trajectory.shape[0]):
        #     future_index = trajectory.shape[0]
        if ( index >= trajectory.shape[0]-100):
            future_index = trajectory.shape[0]-1
        # closest_point=trajectory[index]
        future_point=trajectory[future_index]
        psi_desired = np.arctan2(future_point[1] - Y, future_point[0] - X)

        A=np.array([[0,1,0,0],[0, -4*Ca/(m*xdot),4*Ca/m,-2*Ca*(lf-lr)/(m*xdot)],[0,0,0,1],[0,-2*Ca*(lf-lr)/(Iz*xdot),2*Ca*(lf-lr)/(Iz),-2*Ca*(lf**2+lr**2)/(Iz*xdot)]])
        B=np.array([[0,2*Ca/m,0,2*Ca*lf/Iz]]).transpose()

        poles=np.array([-10,-4,-2,-0.5])
        K = signal.place_poles(A, B, poles).gain_matrix

        # e1= np.linalg.norm(future_point-np.array[X,Y])
        # e1=np.sqrt(np.square(future_point[0]-X)+np.square(future_point[1]-Y))
        e1=dist
        e2=wrapToPi(psi-psi_desired)
        # e1_dot = xdot * np.cos(e2) - ydot * np.sin(e2)
        e1_dot=ydot-(e2*xdot)
        e2_dot=psidot
        state=np.array([e1,e1_dot,e2,e2_dot])
        delta=np.matmul(-K,state)[0]
        if np.absolute(delta)>np.pi/6:
            if delta<0:
                delta=-np.pi/6
            else:
                delta=np.pi/6
        # print(delta)
        # delta=wrapToPi(delta)
        


        # ---------------|Longitudinal Controller|-------------------------
        """
        Please design your longitudinal controller below.
        """
        v_desired=40
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
