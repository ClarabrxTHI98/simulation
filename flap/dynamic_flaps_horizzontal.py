import numpy as np
from control import ss
import os
import sys 
from pathlib import Path

# Add the parent directory to Python path
script_dir = Path(__file__).resolve().parent
parent_dir = script_dir.parent  
sys.path.append(str(parent_dir))

import config

def build_dynamics():
    """
    Builds the state-space dynamics matrices (A, B, C, D)
    Equivalent to the MATLAB script defining the system dynamics.
    """
    # --- Parameters ---#

    # --- Parameters from config ---

    m = config.m
    Ixx = config.Ixx
    Iyy = config.Iyy
    Izz = config.Izz
    g = config.g

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
    B[3, 3] = 1/ Izz           # yaw effect
    B[7, 1] = 1.0 / Ixx         # roll
    B[11, 2] = 1.0 / Iyy        # pitch

    C = np.zeros((4, 12), dtype=float)
    C[0, 0] = 1.0  # z position
    C[1, 2] = 1.0  # x position
    C[2, 10] = 1.0 # phi or related angular state
    C[3, 6] = 1.0  
    D = np.zeros((4, 4), dtype=float)

    sys = ss(A,B,C,D)
    return A, B, C, D, sys

if __name__ == "__main__":
    A, B, C, D, sys = build_dynamics()
    print("A:\n", A)
    print("B:\n", B)
    print("C:\n", C)
    print("D:\n", D)
    print("System created with", A.shape[0], "states and", B.shape[1], "inputs.")
