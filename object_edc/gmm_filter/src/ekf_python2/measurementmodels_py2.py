#!/usr/bin/env python

# Imports
import numpy as np
from numpy import ndarray
from scipy.linalg import block_diag
import rospy
# Measurement models interface declaration



class MeasurementModel:
    def h(self, x, **kwargs):
        """Calculate the noise free measurement location at x in sensor_state.
        Args:
            x (ndarray): state
        """
        raise NotImplementedError

    def H(self, x, **kwargs):
        """Calculate the measurement Jacobian matrix at x in sensor_state.
        Args:
            x (ndarray): state
        """
        raise NotImplementedError

    def R(self, x, **kwargs):
        """Calculate the measurement covariance matrix at x in sensor_state.
        Args:
            x (ndarray): state
        """
        raise NotImplementedError


#@dataclass
#class CartesianPosition2D(MeasurementModel):
#    sigma_z: float
#
#    def h(self, x: ndarray) -> ndarray:
#        """Calculate the noise free measurement location at x in sensor_state.
#        """
#
#        # TODO replace this with your own code
#        first_row = [1, 0, 0, 0]
#        second_row = [0, 1, 0, 0]
#
#        H = np.array([first_row, second_row])
#        x_h = H @ x
#
#        return x_h
#
#    def H(self, x: ndarray) -> ndarray:
#        """Calculate the measurement Jacobian matrix at x in sensor_state."""
#
#        # TODO replace this with your own code
#        first_row = [1, 0, 0, 0]
#        second_row = [0, 1, 0, 0]
#
#        H = np.array([first_row, second_row])
#
#        return H
#
#    def R(self, x: ndarray) -> ndarray:
#        """Calculate the measurement covariance matrix at x in sensor_state."""
#
#        # TODO replace this with your own code
#        identity = np.array([[1, 0], [0, 1]])
#        R = (self.sigma_z**2) * identity
#
#        return R



class NED_range_bearing(MeasurementModel):
    """The model is a sensor agnostic landmark pose measurement model

    state:  x = [pw_wl, eulers_lw] ^ T   a 6x1 vector of position and euler angles
    measurement:    z = [ps_sl, eulers_ls] a 6x1 vector, measurement of full state in sensor frame
    h = euler_from_matrix (R_sl)
    H = diag(R_sw dh / deulers, a 6x6 matrix. Use lambda function Jac_lam to evaluate numerically.
    
    """

    def __init__(self, 
                sigma_sensor, 
                pos, 
                R_sw, 
                R_sl,
                Jac_lam
                ):
        self.sigma_z = sigma_sensor
        self.p_wb = pos
        self.Rot_sw = R_sw
        self.Rot_sl = R_sl
        self.orientation_jacobian = Jac_lam

    
    def h(self, x):
        """Predict measurement through the non-linear vector field h given the
        state x
        x = [pw_wg, gamma_wg] ^ T
        z = [pb_bg, gamma_wg]
        """
         
        z = np.matmul(self.Rot_sw.T, (x[0:3] - self.p_wb[0:3]))

        z_phi = np.arctan2(self.Rot_sl[2,1] , self.Rot_sl[2,2])
        z_theta = np.arcsin(-self.Rot_sl[2,0])
        z_psi = np.arctan2(self.Rot_sl[1,0], self.Rot_sl[0,0])

        z = np.append(z, [z_phi, z_theta, z_psi])

        return z

    def H(self, x):
        """Calculate the measurement Jacobian matrix at x in sensor_state.
        
        x = [pos_x, pos_y , pos_z , phi_x , theta_y , psi_z]
        H_6x6 = [Rot_3x3, 0 
                  0, matrix2euler_jacobian]
        """
        
        euler_state = x[3:6]
        R_sl = self.Rot_sl

        r11, r12, r13, r21, r22, r23, r31, r32, r33 = R_sl[0,0], R_sl[0,1], R_sl[0,2] , R_sl[1,0] , R_sl[1,1] , R_sl[1,2] , R_sl[2,0] , R_sl[2,1] , R_sl[2,2]
        phi_x, theta_y, psi_z = euler_state[0], euler_state[1], euler_state[2]

        H_eulers = self.orientation_jacobian(r11, r12, r13, r21, r22, r23, r31, r32, r33, phi_x, theta_y, psi_z)

        H = block_diag(self.Rot_sw.T, H_eulers)
        return H

    def R(self, x):
        """Calculate the measurement covariance matrix at x in sensor_state."""

        n = len(self.sigma_z)

        R = (self.sigma_z**2) * np.eye(n)

        return R



class NED_semi_linear_landmark(MeasurementModel):
    """The model is a sensor agnostic landmark pose measurement model

    state:  x = [pw_wl, eulers_lw] ^ T   a 6x1 vector of position and euler angles
    measurement:    z = [ps_sl, eulers_ls] a 6x1 vector, measurement of full state in sensor frame
    h = euler_from_matrix (R_sl)
    H = diag(R_sw dh / deulers, a 6x6 matrix. Use lambda function Jac_lam to evaluate numerically.
    
    """

    def __init__(self, 
                sigma_sensor, 
                pos, 
                R_sw, 
                R_sl,
                Jac_lam
                ):
        self.sigma_z = sigma_sensor
        self.p_wb = pos
        self.Rot_sw = R_sw
        self.Rot_sl = R_sl
        self.orientation_jacobian = Jac_lam

    
    def h(self, x):
        """Predict measurement through the non-linear vector field h given the
        state x
        x = [pw_wg, gamma_wg] ^ T
        z = [pb_bg, gamma_wg]
        """
        
        z_pos = np.matmul(self.Rot_sw.T, (x[0:3] - self.p_wb[0:3]))
        z_angs = np.matmul(np.eye(3), x[3:6])
        
        rospy.loginfo(z_pos)
        rospy.loginfo(z_angs)

        z = np.append(z_pos, z_angs)
        return z

    def H(self, x):
        """Calculate the measurement Jacobian matrix at x in sensor_state.
        
        x = [pos_x, pos_y , pos_z , phi_x , theta_y , psi_z]
        H_6x6 = [Rot_3x3, 0 
                  0, matrix2euler_jacobian]
        """
        
        #euler_state = x[3:6]
        #R_sl = self.Rot_sl

        #r11, r12, r13, r21, r22, r23, r31, r32, r33 = R_sl[0,0], R_sl[0,1], R_sl[0,2] , R_sl[1,0] , R_sl[1,1] , R_sl[1,2] , R_sl[2,0] , R_sl[2,1] , R_sl[2,2]
        #phi_x, theta_y, psi_z = euler_state[0], euler_state[1], euler_state[2]

        #H_eulers = self.orientation_jacobian(r11, r12, r13, r21, r22, r23, r31, r32, r33, phi_x, theta_y, psi_z)
        H_eulers = np.eye(3)
        H = block_diag(self.Rot_sw.T, H_eulers)
        return H

    def R(self, x):
        """Calculate the measurement covariance matrix at x in sensor_state."""

        n = len(self.sigma_z)

        R = (self.sigma_z**2) * np.eye(n)

        return R


class NED_linear_landmark(MeasurementModel):
    """The model is a sensor agnostic landmark pose measurement model

    state:  x = [pw_wl, eulers_lw] ^ T   a 6x1 vector of position and euler angles
    measurement:    z = [ps_sl, eulers_ls] a 6x1 vector, measurement of full state in sensor frame
    h = euler_from_matrix (R_sl)
    H = diag(R_sw dh / deulers, a 6x6 matrix. Use lambda function Jac_lam to evaluate numerically.
    
    """

    def __init__(self, 
                sigma_sensor
                ):

        self.sigma_z = sigma_sensor

    
    def h(self, x):
        """Predict measurement through the non-linear vector field h given the
        state prediction x
        x = [pw_wg, gamma_wg] ^ T
        z = [pb_bg, gamma_wg]
        """
        n = np.shape(x)[0]
        z = np.matmul(np.eye(n), x)
        return z

    def H(self, x):
        """Calculate the measurement Jacobian matrix at x in sensor_state.
        
        x = [pos_x, pos_y , pos_z , phi_x , theta_y , psi_z]
        H_6x6 = [Rot_3x3, 0 
                  0, matrix2euler_jacobian]
        """
    
        n = np.shape(x)[0]
        H =  np.eye(n)
        return H

    def R(self, x):
        """Calculate the measurement covariance matrix at x in sensor_state."""

        n = np.shape(self.sigma_z)
        R = (self.sigma_z**2) * np.eye(n)

        return R