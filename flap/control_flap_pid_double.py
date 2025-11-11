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

class PIDFlapControllerCoaxial:


    def __init__(self, Kp_pos=(0,0,0), Kp_ang=(0,0,0), Kp_yaw=(0,0,0)):
        self.Kp_z, self.Ki_z, self.Kd_z = Kp_pos
        self.Kp_ang, self.Ki_ang, self.Kd_ang = Kp_ang
        self.Kp_yaw, self.Ki_yaw, self.Kd_yaw = Kp_yaw
        # Physical parameters
        self.r = config.r_tot
        self.h0 = 0.03
        self.dt = config.dt
        self.m = config.m
        self.g = config.g
        self.k_T = config.k_T       # thrust coefficient
        self.k_m = config.k_Q       # yaw torque coefficient

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

        # -------------------------------
        # Altitude PID → total thrust
        # -------------------------------
        z_err = z_ref - z
        self.integral_z += z_err * self.dt
        derivative_z = (z_err - self.prev_z_err) / self.dt
        self.prev_z_err = z_err
        delta_T = self.Kp_z * z_err + (self.Ki_z or 0.0) * self.integral_z + (self.Kd_z or 0.0) * derivative_z
        T = delta_T  # total thrust command

        # -------------------------------
        # Pitch PID → flap contribution
        # -------------------------------
        pitch_err = pitch_ref - pitch
        self.integral_pitch += pitch_err * self.dt
        derivative_pitch = (pitch_err - self.prev_pitch_err) / self.dt
        self.prev_pitch_err = pitch_err
        delta_h_pitch = self.Kp_ang * pitch_err + (self.Ki_ang or 0.0) * self.integral_pitch + (self.Kd_ang or 0.0) * derivative_pitch

        # -------------------------------
        # Roll PID → flap contribution
        # -------------------------------
        roll_err = roll_ref - roll
        self.integral_roll += roll_err * self.dt
        derivative_roll = (roll_err - self.prev_roll_err) / self.dt
        self.prev_roll_err = roll_err
        delta_h_roll = self.Kp_ang * roll_err + (self.Ki_ang or 0.0) * self.integral_roll + (self.Kd_ang or 0.0) * derivative_roll

        # -------------------------------
        # Yaw PID → rotor speed difference
        # -------------------------------
        yaw_err = yaw_ref - yaw
        self.integral_yaw += yaw_err * self.dt
        derivative_yaw = (yaw_err - self.prev_yaw_err) / self.dt
        self.prev_yaw_err = yaw_err
        tau_yaw_desired = self.Kp_yaw * yaw_err + (self.Ki_yaw or 0.0) * self.integral_yaw + (self.Kd_yaw or 0.0) * derivative_yaw

        # -------------------------------
        # Coaxial rotor speeds
        # Solve:
        # T = k_T*(omega_top^2 + omega_bottom^2)
        # tau_yaw_desired = k_m*(omega_top^2 - omega_bottom^2)
        # -------------------------------
        omega_top_sq = (T/self.k_T + tau_yaw_desired/self.k_m)/2
        omega_bottom_sq = (T/self.k_T - tau_yaw_desired/self.k_m)/2

        # Avoid negative sqrt
        omega_top = np.sqrt(abs(omega_top_sq))
        omega_bottom = np.sqrt(abs(omega_bottom_sq))


        # Actual yaw torque from these speeds
        tau_yaw = self.k_m * (omega_top**2 - omega_bottom**2)

        # -------------------------------
        # Flap heights → pitch/roll only
        # -------------------------------
        h1 = self.h0 - delta_h_pitch
        h2 = self.h0 - delta_h_roll
        h3 = self.h0 + delta_h_pitch
        h4 = self.h0 + delta_h_roll

        # Torques from flap heights
        tau_roll = 64 * self.r * (h4 - h2)
        tau_pitch = 64 * self.r * (h3 - h1)

        return {
            'T': T,
            'tau_pitch': tau_pitch,
            'tau_roll': tau_roll,
            'tau_yaw': tau_yaw,
            'omega_top': omega_top,
            'omega_bottom': omega_bottom,
            'h1': h1, 'h2': h2, 'h3': h3, 'h4': h4
        }