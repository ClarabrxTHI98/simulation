'''
import numpy as np
import config

class PIDFlapController:

    def __init__(self, Kp_pos, Ki_pos, Kd_pos, Kp_ang, Ki_ang, Kd_ang, Kp_yaw, Ki_yaw, Kd_yaw):
        # Gains
        self.Kp_z, self.Ki_z, self.Kd_z = Kp_pos
        self.Kp_ang, self.Ki_ang, self.Kd_ang = Kp_ang
        self.Kp_yaw, self.Ki_yaw, self.Kd_yaw = Kp_yaw, Ki_yaw, Kd_yaw

        # Physical params (from config)
        self.r = config.r_tot
        self.h0 = 0.03
        self.dt = config.dt
        self.m = config.m
        self.g = config.g
        self.k_T = config.k_T  # thrust coefficient

        # Yaw aero params
        self.k_m = config.k_Q
        self.rho = config.rho
        self.k_s = config.ks
        self.A_r = config.Ar
        self.d = config.d_int
        self.k_a = config.k_alpha
        self.Ar = config.Ar
        self.n = config.n

        # Integrals
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
        Compute control input from desired positions and angles.
        Returns a dictionary with thrust, torques, and flap heights.
        """

        # -------------------------------
        # Altitude (z) PID
        # -------------------------------
        z_err = z_ref - z
        self.integral_z += z_err * self.dt
        derivative_z = (z_err - self.prev_z_err) / self.dt
        self.prev_z_err = z_err
        delta_T = self.Kp_z * z_err + (self.Ki_z or 0.0) * self.integral_z + (self.Kd_z or 0.0) * derivative_z

        # Total thrust including gravity compensation
        #T = -(self.m * self.g) + delta_T
        T = delta_T

        # -------------------------------
        # Compute dynamic rotor speed
        # -------------------------------
        omega = np.sqrt(max(T / self.k_T, 0.0))  # rotor speed cannot be negative

        # -------------------------------
        # Pitch PID
        # -------------------------------
        pitch_err = pitch_ref - pitch
        self.integral_pitch += pitch_err * self.dt
        derivative_pitch = (pitch_err - self.prev_pitch_err) / self.dt
        self.prev_pitch_err = pitch_err
        delta_h_pitch = self.Kp_ang * pitch_err + (self.Ki_ang or 0.0) * self.integral_pitch + (self.Kd_ang or 0.0) * derivative_pitch

        # -------------------------------
        # Roll PID
        # -------------------------------
        roll_err = roll_ref - roll
        self.integral_roll += roll_err * self.dt
        derivative_roll = (roll_err - self.prev_roll_err) / self.dt
        self.prev_roll_err = roll_err
        delta_h_roll = self.Kp_ang * roll_err + (self.Ki_ang or 0.0) * self.integral_roll + (self.Kd_ang or 0.0) * derivative_roll

        # -------------------------------
        # Yaw PID (passive + optional active)
        # -------------------------------
        yaw_err = yaw_ref - yaw
        self.integral_yaw += yaw_err * self.dt
        derivative_yaw = (yaw_err - self.prev_yaw_err) / self.dt
        self.prev_yaw_err = yaw_err

        alpha = 0.52  # static angle of attack for yaw vanes
        tau_yaw = omega**2 * (self.k_m + self.rho*self.k_s**2*self.A_r*self.d - self.n*self.k_a*alpha*self.rho*self.k_s**2/2*self.Ar)

        # -------------------------------
        # Flap heights
        # -------------------------------
        h1 = self.h0 - delta_h_pitch
        h2 = self.h0 - delta_h_roll
        h3 = self.h0 + delta_h_pitch
        h4 = self.h0 + delta_h_roll

        # -------------------------------
        # Torques from flap heights
        # -------------------------------
        tau_roll = 64 * self.r * (h4 - h2)
        tau_pitch = 64 * self.r * (h3 - h1)

        return {
            'T': T,
            'tau_pitch': tau_pitch,
            'tau_roll': tau_roll,
            'tau_yaw': tau_yaw,
            'omega': omega,          # dynamic rotor speed
            'alpha_vane': alpha,     # for logging
            'h1': h1, 'h2': h2, 'h3': h3, 'h4': h4
        }

import numpy as np
import config

class PIDFlapController:

    def __init__(self, Kp_pos, Ki_pos, Kd_pos, Kp_ang, Ki_ang, Kd_ang, Kp_yaw, Ki_yaw, Kd_yaw):
        # Gains
        self.Kp_z, self.Ki_z, self.Kd_z = Kp_pos
        self.Kp_ang, self.Ki_ang, self.Kd_ang = Kp_ang
        self.Kp_yaw, self.Ki_yaw, self.Kd_yaw = Kp_yaw, Ki_yaw, Kd_yaw

        # Physical params (from config)
        self.r = config.r_tot
        self.h0 = 0.03
        self.dt = config.dt
        self.m = config.m
        self.g = config.g
        self.k_T = config.k_T  # thrust coefficient

        # Yaw aero params
        self.k_m = config.k_Q
        self.rho = config.rho
        self.k_s = config.ks
        self.A_r = config.Ar
        self.d = config.d_int
        self.k_a = config.k_alpha
        self.Ar = config.Ar
        self.n = config.n

        # Integrals
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
        Compute control input from desired positions and angles.
        Automatically adjusts alpha (angle of attack of yaw vanes) to achieve tau_yaw = 0.
        """

        # -------------------------------
        # Altitude (z) PID
        # -------------------------------
        z_err = z_ref - z
        self.integral_z += z_err * self.dt
        derivative_z = (z_err - self.prev_z_err) / self.dt
        self.prev_z_err = z_err
        delta_T = self.Kp_z * z_err + (self.Ki_z or 0.0) * self.integral_z + (self.Kd_z or 0.0) * derivative_z

        # Total thrust
        T = delta_T
        T_for_omega = np.absolute(delta_T)

        # -------------------------------
        # Dynamic rotor speed
        # -------------------------------
        omega = np.sqrt(T_for_omega / self.k_T)

        # -------------------------------
        # Pitch PID
        # -------------------------------
        pitch_err = pitch_ref - pitch
        self.integral_pitch += pitch_err * self.dt
        derivative_pitch = (pitch_err - self.prev_pitch_err) / self.dt
        self.prev_pitch_err = pitch_err
        delta_h_pitch = self.Kp_ang * pitch_err + (self.Ki_ang or 0.0) * self.integral_pitch + (self.Kd_ang or 0.0) * derivative_pitch

        # -------------------------------
        # Roll PID
        # -------------------------------
        roll_err = roll_ref - roll
        self.integral_roll += roll_err * self.dt
        derivative_roll = (roll_err - self.prev_roll_err) / self.dt
        self.prev_roll_err = roll_err
        delta_h_roll = self.Kp_ang * roll_err + (self.Ki_ang or 0.0) * self.integral_roll + (self.Kd_ang or 0.0) * derivative_roll

        # -------------------------------
        # Compute alpha for tau_yaw = 0
        # -------------------------------
        numerator = 2 * (self.k_m + self.rho * self.k_s**2 * self.A_r * self.d)
        denominator = self.n * self.k_a * self.rho * self.k_s**2 * self.Ar
        alpha = numerator / denominator  # automatic correction angle of attack

        # -------------------------------
        # Verify resulting yaw torque
        # -------------------------------
        tau_yaw = omega**2 * (
            self.k_m + self.rho * self.k_s**2 * self.A_r * self.d -
            self.n * self.k_a * alpha * self.rho * self.k_s**2 / 2 * self.Ar
        )

        # -------------------------------
        # Flap heights
        # -------------------------------
        h1 = self.h0 - delta_h_pitch
        h2 = self.h0 - delta_h_roll
        h3 = self.h0 + delta_h_pitch
        h4 = self.h0 + delta_h_roll

        # -------------------------------
        # Torques from flap heights
        # -------------------------------
        tau_roll = 64 * self.r * (h4 - h2)
        tau_pitch = 64 * self.r * (h3 - h1)

        return {
            'T': T,
            'tau_pitch': tau_pitch,
            'tau_roll': tau_roll,
            'tau_yaw': tau_yaw,  # should be ≈ 0
            'omega': omega,
            'alpha_vane': alpha,  # computed automatically
            'h1': h1, 'h2': h2, 'h3': h3, 'h4': h4
        }
'''

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

    def __init__(self, Kp_pos, Ki_pos, Kd_pos, Kp_ang, Ki_ang, Kd_ang, Kp_yaw, Ki_yaw, Kd_yaw,
                 vane_csv_path=None):
        # PID gains
        self.Kp_z, self.Ki_z, self.Kd_z = Kp_pos
        self.Kp_ang, self.Ki_ang, self.Kd_ang = Kp_ang
        self.Kp_yaw, self.Ki_yaw, self.Kd_yaw = Kp_yaw, Ki_yaw, Kd_yaw

        # Physical parameters
        self.r = config.r_tot
        self.h0 = 0.03
        self.dt = config.dt
        self.m = config.m
        self.g = config.g
        self.k_T = config.k_T
        self.k_m = config.k_Q

        # Yaw vane parameters
        self.rho = config.rho
        self.k_s = config.ks
        self.A_r = config.Ar
        self.d = config.d_int
        self.n = config.n
        self.l = 0.1   # flap span
        self.c = 0.02  # flap chord

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

        # Load airfoil polar for Cl(alpha)
        if vane_csv_path is not None:
            df = pd.read_csv(vane_csv_path, skiprows=9)  # skip header to get to data
            self.alpha_csv = df['Alpha'].values * np.pi/180  # convert deg → rad
            self.Cl_csv = df['Cl'].values
        else:
            self.alpha_csv = None
            self.Cl_csv = None

    def step(self, x=0, y=0, z=0, pitch=0, roll=0, yaw=0,
             x_ref=0, y_ref=0, z_ref=0, pitch_ref=0, roll_ref=0, yaw_ref=0):
        """
        Compute control input.
        Computes alpha_vane using aerodynamic lift to cancel yaw torque.
        """

        # -------------------------------
        # Altitude PID
        # -------------------------------
        z_err = z_ref - z
        self.integral_z += z_err * self.dt
        derivative_z = (z_err - self.prev_z_err) / self.dt
        self.prev_z_err = z_err
        delta_T = self.Kp_z * z_err + (self.Ki_z or 0.0) * self.integral_z + (self.Kd_z or 0.0) * derivative_z
        T = delta_T
        omega = np.sqrt(abs(T) / self.k_T)  # rotor speed

        # -------------------------------
        # Pitch PID
        # -------------------------------
        pitch_err = pitch_ref - pitch
        self.integral_pitch += pitch_err * self.dt
        derivative_pitch = (pitch_err - self.prev_pitch_err) / self.dt
        self.prev_pitch_err = pitch_err
        delta_h_pitch = self.Kp_ang * pitch_err + (self.Ki_ang or 0.0) * self.integral_pitch + (self.Kd_ang or 0.0) * derivative_pitch

        # -------------------------------
        # Roll PID
        # -------------------------------
        roll_err = roll_ref - roll
        self.integral_roll += roll_err * self.dt
        derivative_roll = (roll_err - self.prev_roll_err) / self.dt
        self.prev_roll_err = roll_err
        delta_h_roll = self.Kp_ang * roll_err + (self.Ki_ang or 0.0) * self.integral_roll + (self.Kd_ang or 0.0) * derivative_roll

        # -------------------------------
        # Compute vane angle alpha to cancel tau_yaw
        # -------------------------------
        # Induced air velocity in vane channel
        V = self.k_s * omega
        q = 0.5 * self.rho * V**2  # dynamic pressure

        # Solve tau_yaw = tau_rotor - tau_vane = 0 → find alpha
        tau_rotor = self.k_m * omega**2
        if self.alpha_csv is not None:
            # Function to compute tau_vane for a given alpha
            def tau_vane_func(alpha):
                Cl = np.interp(alpha, self.alpha_csv, self.Cl_csv)
                S = self.l * self.c * self.n
                return Cl * q * S * self.r

            # Simple numerical root-finding (bisection)
            alpha_min, alpha_max = -np.pi/6, np.pi/6  # ±30 deg
            alpha_vane = 0.0
            for _ in range(20):  # 20 iterations
                tau_v = tau_vane_func(alpha_vane)
                error = tau_rotor - tau_v
                if abs(error) < 1e-6:
                    break
                alpha_vane += np.clip(error/(q*S*self.r), -0.01, 0.01)  # step proportional to torque error
        else:
            # fallback
            numerator = 2 * (self.k_m + self.rho * self.k_s**2 * self.A_r * self.d)
            denominator = self.n * self.rho * self.k_s**2 * self.A_r
            alpha_vane = numerator / denominator

        # Compute resulting tau_yaw
        tau_vane = np.interp(alpha_vane, self.alpha_csv, self.Cl_csv) * q * self.l * self.c * self.n * self.r if self.alpha_csv is not None else tau_rotor
        tau_yaw = tau_rotor - tau_vane

        # -------------------------------
        # Flap heights
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
            'omega': omega,
            'alpha_vane': alpha_vane,
            'h1': h1, 'h2': h2, 'h3': h3, 'h4': h4
        }
