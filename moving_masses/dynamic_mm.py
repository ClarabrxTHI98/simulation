# dynamic.py
import numpy as np
import config

def dynamic_system_mm_4dof(y, control=None, params=None):
    # everything else stays the same

    """
    4-DOF moving-mass + rotor system dynamics using parameters from config.py
    State vector y = [z, dz, x, dx, theta, dtheta, ll, dll, lr, dlr, eps, deps, phi, dphi]
    Control dictionary can include:
        control = {
            'T': thrust along z (float),
            'F_x': optional horizontal force,
            'tau_theta': optional torque on theta,
            ...
        }
    """

    # --- Parameters from config ---
    m = config.m
    m_l = config.m_l
    m_r = config.m_r
    I_rot = config.Izz
    J_l = config.J_l
    J_r = config.J_r
    ll_1 = 0.3 #config.ll_1
    lr_1 = -0.3 #config.lr_1
    d_mm = config.d_mm
    g = config.g
    m_tot = m + m_l + m_r

    # --- Unpack state ---
    z_1, z_2, x_1, x_2, theta_1, theta_2, ll_1, ll_2, lr_1, lr_2, eps_1, eps_2, phi_1, phi_2 = y
    # --- Override moving-mass positions if provided in control ---
    if control is not None:
        ll_1 = control.get('ll', 0.3)   # keep current if not provided
        lr_1 = control.get('lr', 0.3)
        eps_1 = control.get('eps', 0)
        phi_1 = control.get('phi', 0)
        T_applied = control.get('T', 0.0)  # default thrust = 0
    else:
        T_applied = 0.0

    # --- Mass matrix M (7x7) ---
    M = np.array([
        [m_tot, 0,  (2*d_mm*np.sin(theta_1)*m_r/2)+(2*d_mm*np.sin(theta_1)*m_l/2), 2*np.sin(eps_1)*m_l/2, -2*np.sin(phi_1)*m_r/2, m_l/2*(2*ll_1*np.cos(eps_1)), -m_r/2*(2*lr_1*np.cos(phi_1))],
        [0,  m_tot,  (-2*d_mm*np.cos(theta_1)*m_r/2-(2*d_mm*np.cos(theta_1)*m_l/2)), -2*np.cos(eps_1)*m_l/2, 2*np.cos(phi_1)*m_r/2, m_l/2*(2*ll_1*np.sin(eps_1)), -m_r/2*(2*lr_1*np.sin(phi_1))],
        [(2*d_mm*np.sin(theta_1)*m_r/2)+(2*d_mm*np.sin(theta_1)*m_l/2), ((-2*d_mm*np.cos(theta_1))*m_r/2+(-2*d_mm*np.cos(theta_1))*m_l/2), (m_r*d_mm**2 + m_l*d_mm**2 + I_rot + J_l + J_r), m_l/2*(2*d_mm*np.cos(theta_1)*np.cos(eps_1) + 2*d_mm*np.sin(theta_1)*np.sin(eps_1)), m_r/2*(-2*d_mm*np.cos(theta_1)*np.cos(phi_1)-2*d_mm*np.sin(theta_1)*np.sin(phi_1)), m_l/2*(-2*d_mm*ll_1*np.cos(theta_1)*np.sin(eps_1)+2*d_mm*ll_1*np.sin(theta_1)*np.cos(eps_1))+J_l, m_r/2*(2*d_mm*lr_1*np.cos(theta_1)*np.sin(phi_1)-2*d_mm*lr_1*np.sin(theta_1)*np.cos(phi_1))+J_r],
        [2*np.sin(eps_1)*m_l/2, -2*np.cos(eps_1)*m_l/2, m_l/2*(2*d_mm*np.cos(theta_1)*np.cos(eps_1)+2*d_mm*np.sin(theta_1)*np.sin(eps_1)), m_l, 0, 0, 0],
        [-2*np.sin(phi_1)*m_r/2, 2*np.cos(phi_1)*m_r/2, m_r/2*(-2*d_mm*np.cos(theta_1)*np.cos(phi_1)-2*d_mm*np.sin(theta_1)*np.sin(phi_1)), 0, m_r, 0, 0],
        [m_l/2*(2*ll_1*np.cos(eps_1)), m_l/2*(2*ll_1*np.sin(eps_1)), m_l/2*(-2*d_mm*ll_1*np.cos(theta_1)*np.sin(eps_1)+2*d_mm*ll_1*np.sin(theta_1)*np.cos(eps_1)) + J_l, 0, 0, m_l*ll_1**2 + J_l, 0],
        [-m_r/2*(2*lr_1*np.cos(phi_1)), -m_r/2*(2*lr_1*np.sin(phi_1)), m_r/2*(2*d_mm*lr_1*np.cos(theta_1)*np.sin(phi_1) -2*d_mm*lr_1*np.sin(theta_1)*np.cos(phi_1)) + J_r, 0, 0, 0, m_r*lr_1**2 + J_r]
    ])

    # --- Coriolis / centripetal matrix C (7x7) ---
    C = np.zeros((7,7))  # Can extend later if needed

    # --- Gravity vector ---
    dV = np.array([
        m_tot*g,
        0,
        (m_l*d_mm*np.sin(theta_1) + m_r*d_mm*np.sin(theta_1))*g,
        m_l*g*np.sin(eps_1),
        -m_r*g*np.sin(phi_1),
        m_l*g*ll_1*np.cos(eps_1),
        -m_r*g*lr_1*np.cos(phi_1)
    ])

 
    # You can add more control components here if needed
    G = np.array([
        T_applied,
        0,
        0,
        0,
        0,
        0,
        0
    ])

    # --- Velocity vector ---
    vel_vector = np.array([z_2, x_2, theta_2, ll_2, lr_2, eps_2, phi_2])

    # --- Solve for accelerations ---
    accelerations = np.linalg.solve(M, -C @ vel_vector + G - dV)
    z_2_dot, x_2_dot, theta_2_dot, ll_2_dot, lr_2_dot, eps_2_dot, phi_2_dot = accelerations

    # --- Return dydt ---
    dydt = np.array([
        z_2, z_2_dot,
        x_2, x_2_dot,
        theta_2, theta_2_dot,
        ll_2, ll_2_dot,
        lr_2, lr_2_dot,
        eps_2, eps_2_dot,
        phi_2, phi_2_dot
    ])

    return dydt
