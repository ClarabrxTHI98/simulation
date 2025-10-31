import numpy as np
from control import ss  # optional if you want to build a state-space object
import config

def build_dynamics():
    """
    Builds the state-space dynamics matrices (A, B, C, D)
    Equivalent to the MATLAB script defining the system dynamics.
    """
    # --- Parameters ---#

    # --- Parameters from config ---
    m = config.m
    I_zz = config.Izz
    I_xx = config.Ixx
    I_yy = config.Iyy

    d_mm = config.d_mm
    g = config.g
    E = config.E

    # --- System matrices ---
    A = np.array([
        [0, 1, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0,  0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1,  0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, -g, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0,  0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0,  0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0,  0, 0, 0, 0, g, 0],
        [0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0]
    ], dtype=float)

    B = np.zeros((12, 4), dtype=float)
    B[1, 0] = 1.0 / m         # thrust input
    B[3, 3] = 1/ I_zz           # yaw effect
    B[7, 1] = 1.0 / I_xx         # roll
    B[11, 2] = 1.0 / I_yy        # pitch

    C = np.zeros((4, 12), dtype=float)
    C[0, 0] = 1.0  # z position
    C[1, 2] = 1.0  # x position
    C[2, 10] = 1.0 # phi or related angular state
    C[3, 6] = 1.0  
    D = np.zeros((4, 4), dtype=float)

    # Optionally return both raw matrices and a control.ss() system
    sys = ss(A, B, C, D)
    return A, B, C, D, sys

if __name__ == "__main__":
    A, B, C, D, sys = build_dynamics()
    print("A:\n", A)
    print("B:\n", B)
    print("C:\n", C)
    print("D:\n", D)
    print("System created with", A.shape[0], "states and", B.shape[1], "inputs.")


