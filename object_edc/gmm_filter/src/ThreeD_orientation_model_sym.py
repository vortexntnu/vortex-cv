#from numba import njit
import numpy as np
from sympy import *
from sympy.vector import CoordSys3D, SpaceOrienter
import sympy 
sp=sympy


def init():

    # Time varying matrix giving the transformation from world to body
    r11, r12, r13, r21, r22, r23, r31, r32, r33 = symbols (" r11, r12, r13, r21, r22, r23, r31, r32, r33 ")
    R_cw = Matrix([
        [r11, r12, r13], [r21, r22, r23], [r31, r32, r33]
    ])

    ## Trying with euler angles
    phi_lw, theta_lw, psi_lw = symbols("phi_lw, theta_lw, psi_lw")
    landmark_space_orienter = SpaceOrienter(phi_lw, theta_lw, psi_lw, "123") # This can go very wrong

    N = CoordSys3D("N")
    B = N.orient_new("B", (landmark_space_orienter))

    R_lw = N.rotation_matrix(B)
    R_wl = R_lw.T

    R_cl = R_cw * R_wl

    # This can go very wrong
    phi_x = sympy.atan2(R_cl[2,1] , R_cl[2,2])
    theta_y = sympy.asin(-R_cl[2,0])
    psi_z = sympy.atan2(R_cl[1,0], R_cl[0,0])

    state = Matrix([phi_lw, theta_lw, psi_lw])
    h = Matrix([phi_x, theta_y, psi_z])
    
    Jac = h.jacobian(state)
    
    #return njit(lambdify([r11, r12, r13, r21, r22, r23, r31, r32, r33, phi_lw, theta_lw, psi_lw], Jac, 'numpy'))
    return lambdify([r11, r12, r13, r21, r22, r23, r31, r32, r33, phi_lw, theta_lw, psi_lw], Jac, 'numpy')