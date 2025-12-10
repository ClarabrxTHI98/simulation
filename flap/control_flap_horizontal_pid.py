import numpy as np
import pandas as pd
import os
import sys
from pathlib import Path

# Add the parent directory to Python path
script_dir = Path(__file__).resolve().parent
parent_dir = script_dir.parent
sys.path.append(str(parent_dir))

import config

class PIDFlapController:
    """
    PID-based controller for a single-rotor vehicle with 4 flaps whose
    rotational axes are parallel to the z-axis.
    Each flap deflection now affects pitch, roll, and yaw torques simultaneously.
    """

    def __init__(self, Kp_pos, Ki_pos, Kd_pos, Kp_ang, Ki_ang, Kd_ang, Kp_yaw, Ki_yaw, Kd_yaw):
        # PID gains
        self.Kp_z, self.Ki_z, self.Kd_z = Kp_pos
        self.Kp_ang, self.Ki_ang, self.Kd_ang = Kp_ang
        self.Kp_yaw, self.Ki_yaw, self.Kd_yaw = Kp_yaw, Ki_yaw, Kd_yaw

        # Physical parameters
        self.r = config.r_tot          # effective lever arm
        self.h0 = 0.03                 # nominal flap deflection
        self.dt = config.dt
        self.m = config.m
        self.g = config.g
        self.k_T = config.k_T
        self.k_m = config.k_Q          # rotor torque constant

        # Aerodynamic coupling constants
        self.k_tau_pitch = config.k_tau_pitch if hasattr(config, "k_tau_pitch") else 50.0
        self.k_tau_roll  = config.k_tau_roll  if hasattr(config, "k_tau_roll") else 50.0
        self.k_tau_yaw   = config.k_tau_yaw   if hasattr(config, "k_tau_yaw") else 5.0  # weaker yaw coupling

        # PID integrators
        self.integral_z = 0.0
        self.integral_pitch = 0.0
        self.integral_roll = 0.0
        self.integral_yaw = 0.0

        # Previous errors
        self.prev_z_err = 0.0
        self.prev_pitch_err = 0.0
        self.prev_roll_err = 0.0
        self.prev_yaw_err = 0.0

    def step(self, x=0, y=0, z=0, pitch=0, roll=0, yaw=0,
             x_ref=0, y_ref=0, z_ref=0, pitch_ref=0, roll_ref=0, yaw_ref=0):

        """
        Compute control input (thrust + torques) for the flap system.
        Each flap is assumed to influence roll, pitch, and yaw simultaneously.
        """

        # -------------------------------
        # Altitude PID
        # -------------------------------
        
        z_err = z_ref - z
        self.integral_z += z_err * self.dt
        derivative_z = (z_err - self.prev_z_err) / self.dt
        self.prev_z_err = z_err

        delta_T = self.Kp_z * z_err + self.Ki_z * self.integral_z + self.Kd_z * derivative_z
        T = self.m * self.g + delta_T  # total thrust command

        # -------------------------------
        # Pitch PID
        # -------------------------------
        pitch_err = pitch_ref - pitch
        self.integral_pitch += pitch_err * self.dt
        derivative_pitch = (pitch_err - self.prev_pitch_err) / self.dt
        self.prev_pitch_err = pitch_err

        delta_h_pitch = self.Kp_ang * pitch_err + self.Ki_ang * self.integral_pitch + self.Kd_ang * derivative_pitch

        # -------------------------------
        # Roll PID
        # -------------------------------
        roll_err = roll_ref - roll
        self.integral_roll += roll_err * self.dt
        derivative_roll = (roll_err - self.prev_roll_err) / self.dt
        self.prev_roll_err = roll_err

        delta_h_roll = self.Kp_ang * roll_err + self.Ki_ang * self.integral_roll + self.Kd_ang * derivative_roll

        # -------------------------------
        # Yaw PID
        # -------------------------------
        yaw_err = yaw_ref - yaw
        self.integral_yaw += yaw_err * self.dt
        derivative_yaw = (yaw_err - self.prev_yaw_err) / self.dt
        self.prev_yaw_err = yaw_err

        delta_h_yaw = self.Kp_yaw * yaw_err + self.Ki_yaw * self.integral_yaw + self.Kd_yaw * derivative_yaw

        # -------------------------------
        # Combine flap commands
        # -------------------------------
        # Each flap influences roll, pitch, and yaw:
        # h_i = h0 + a*pitch + b*roll + c*yaw (signs depend on position)
        h1 = self.h0 + delta_h_pitch - delta_h_roll + delta_h_yaw   # front-left
        h2 = self.h0 + delta_h_pitch + delta_h_roll - delta_h_yaw   # front-right
        h3 = self.h0 - delta_h_pitch + delta_h_roll + delta_h_yaw   # rear-right
        h4 = self.h0 - delta_h_pitch - delta_h_roll - delta_h_yaw   # rear-left

        # -------------------------------
        # Compute torques from flap deflection
        # -------------------------------
        tau_pitch = self.k_tau_pitch * ((h1 + h2) - (h3 + h4)) / 4.0
        tau_roll  = self.k_tau_roll  * ((h2 + h3) - (h1 + h4)) / 4.0
        tau_yaw   = self.k_tau_yaw   * ((h1 - h2) + (h3 - h4)) / 4.0

        return {
            'T': T,
            'tau_pitch': tau_pitch,
            'tau_roll': tau_roll,
            'tau_yaw': tau_yaw,
            'h1': h1, 'h2': h2, 'h3': h3, 'h4': h4
        }
