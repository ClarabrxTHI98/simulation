import numpy as np

def quat_omega_matrix(omega):
    wx, wy, wz = omega
    return np.array([
        [0.0, -wx, -wy, -wz],
        [wx, 0.0, wz, -wy],
        [wy, -wz, 0.0, wx],
        [wz, wy, -wx, 0.0]
    ])

def normalize_quat(q):
    return q / np.linalg.norm(q)

def dynamics(state, control, params):
    m = params['m']
    I = params['I']
    g = params['g']

    # unpack
    r = state[0:3]
    v = state[3:6]
    q = state[6:10]
    omega = state[10:13]

    # Normalize quaternion to prevent overflow
    q_norm = np.linalg.norm(q)
    if q_norm == 0:
        q = np.array([1.0, 0.0, 0.0, 0.0])
    else:
        q = q / q_norm

    q0, q1, q2, q3 = q
    R = np.array([
        [1-2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3), 1-2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1-2*(q1**2 + q2**2)]
    ])

    F_b = np.array([0.0, 0.0, control['T']])
    a = (1.0/m) * (m*np.array([0,0,-g]) + R @ F_b)

    Omega = quat_omega_matrix(omega)
    q_dot = 0.5 * Omega.dot(q)

    tau = control['tau']
    I_omega = I.dot(omega)
    omega_dot = np.linalg.inv(I).dot(tau - np.cross(omega, I_omega))

    deriv = np.zeros_like(state)
    deriv[0:3] = v
    deriv[3:6] = a
    deriv[6:10] = q_dot
    deriv[10:13] = omega_dot

    return deriv
