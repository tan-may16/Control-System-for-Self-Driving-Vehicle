# Fill in the respective function to implement the LQR/EKF SLAM controller

# Import libraries
import numpy as np
from base_controller import BaseController
from scipy import signal, linalg
from scipy.spatial.transform import Rotation
from util import *
from ekf_slam import EKF_SLAM

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
        
        self.counter = 0
        np.random.seed(99)

        # Add additional member variables according to your need here.

    def getStates(self, timestep, use_slam=False):

        delT, X, Y, xdot, ydot, psi, psidot = super().getStates(timestep)

        # Initialize the EKF SLAM estimation
        if self.counter == 0:
            # Load the map
            minX, maxX, minY, maxY = -120., 450., -500., 50.
            map_x = np.linspace(minX, maxX, 7)
            map_y = np.linspace(minY, maxY, 7)
            map_X, map_Y = np.meshgrid(map_x, map_y)
            map_X = map_X.reshape(-1,1)
            map_Y = map_Y.reshape(-1,1)
            self.map = np.hstack((map_X, map_Y)).reshape((-1))
            
            # Parameters for EKF SLAM
            self.n = int(len(self.map)/2)             
            X_est = X + 0.5
            Y_est = Y - 0.5
            psi_est = psi - 0.02
            mu_est = np.zeros(3+2*self.n)
            mu_est[0:3] = np.array([X_est, Y_est, psi_est])
            mu_est[3:] = np.array(self.map)
            init_P = 1*np.eye(3+2*self.n)
            W = np.zeros((3+2*self.n, 3+2*self.n))
            W[0:3, 0:3] = delT**2 * 0.1 * np.eye(3)
            V = 0.1*np.eye(2*self.n)
            V[self.n:, self.n:] = 0.01*np.eye(self.n)
            # V[self.n:] = 0.01
            print(V)
            
            # Create a SLAM
            self.slam = EKF_SLAM(mu_est, init_P, delT, W, V, self.n)
            self.counter += 1
        else:
            mu = np.zeros(3+2*self.n)
            mu[0:3] = np.array([X, 
                                Y, 
                                psi])
            mu[3:] = self.map
            y = self._compute_measurements(X, Y, psi)
            mu_est, _ = self.slam.predict_and_correct(y, self.previous_u)

        self.previous_u = np.array([xdot, ydot, psidot])

        print("True      X, Y, psi:", X, Y, psi)
        print("Estimated X, Y, psi:", mu_est[0], mu_est[1], mu_est[2])
        print("-------------------------------------------------------")
        
        if use_slam == True:
            return delT, mu_est[0], mu_est[1], xdot, ydot, mu_est[2], psidot
        else:
            return delT, X, Y, xdot, ydot, psi, psidot

    def _compute_measurements(self, X, Y, psi):
        x = np.zeros(3+2*self.n)
        x[0:3] = np.array([X, Y, psi])
        x[3:] = self.map
        
        p = x[0:2]
        psi = x[2]
        m = x[3:].reshape((-1,2))

        y = np.zeros(2*self.n)

        for i in range(self.n):
            y[i] = np.linalg.norm(m[i, :] - p)
            y[self.n+i] = wrapToPi(np.arctan2(m[i,1]-p[1], m[i,0]-p[0]) - psi)
            
        y = y + np.random.multivariate_normal(np.zeros(2*self.n), self.slam.V)
        # print(np.random.multivariate_normal(np.zeros(2*self.n), self.slam.V))
        return y

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
        # Fetch the states from the newly defined getStates method
        delT, X, Y, xdot, ydot, psi, psidot = self.getStates(timestep, use_slam=True)
        # You must not use true_X, true_Y and true_psi since they are for plotting purpose
        _, true_X, true_Y, _, _, true_psi, _ = self.getStates(timestep, use_slam=False)

        # You are free to reuse or refine your code from P3 in the spaces below.

        # ---------------|Lateral Controller|-------------------------
        dist, index = closestNode(X, Y, trajectory)
        future_index=index+130
        # if ( future_index >= trajectory.shape[0]):
        #     future_index = trajectory.shape[0]
        if ( index >= trajectory.shape[0]-130):
            future_index = trajectory.shape[0]-1
        # closest_point=trajectory[index]
        future_point=trajectory[future_index]
        psi_desired = np.arctan2(future_point[1] - Y, future_point[0] - X)

        A=np.array([[0,1,0,0],[0, -4*Ca/(m*xdot),4*Ca/m,-2*Ca*(lf-lr)/(m*xdot)],[0,0,0,1],[0,-2*Ca*(lf-lr)/(Iz*xdot),2*Ca*(lf-lr)/(Iz),-2*Ca*(lf**2+lr**2)/(Iz*xdot)]])
        B=np.array([[0,2*Ca/m,0,2*Ca*lf/Iz]]).transpose()
        C=np.eye(4)
        D=np.zeros(4)
        constants=(A,B,C,D)
        # print(delT)
        Ad,Bd,Cd,Dd,_=signal.cont2discrete(constants,delT)

        e1=dist
        e2=wrapToPi(psi-psi_desired)
        e1_dot=ydot-(e2*xdot)
        e2_dot=psidot
        state=np.array([e1,e1_dot,e2,e2_dot])

        # Q=np.array([[1,1,1,1],[1,1,1,1],[1,1,1,1],[1,1,1,1]])
        Q=np.array([[0.1,0,0,0],[0,1,0,0],[0,0,0.1,0],[0,0,0,1]])
        # print(Q.shape)
        R=np.array([[200]])
        # print(R.shape)
        S = np.matrix(linalg.solve_discrete_are(Ad, Bd, Q, R))
        K = np.matrix(linalg.inv(Bd.T@S@Bd+R)@(Bd.T@S@Ad))

        delta=np.matmul(-K,state)
        # print(delta.shape)
        delta=delta[0,0]
        
        # print(delta)
        delta=wrapToPi(delta)
        
        # ---------------|Longitudinal Controller|-------------------------
        """
        Please design your longitudinal controller below.
        
        """
        v_desired=35
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
        return true_X, true_Y, xdot, ydot, true_psi, psidot, F, delta
