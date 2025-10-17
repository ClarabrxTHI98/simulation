import numpy as np

def rk4_step(f, state, control, params, dt):
    k1 = f(state, control, params)
    k2 = f(state + 0.5*dt*k1, control, params)
    k3 = f(state + 0.5*dt*k2, control, params)
    k4 = f(state + dt*k3, control, params)
    return state + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)

def simulate(f, initial_state, control_trajectory, params, dt):
    N = len(control_trajectory)
    traj = []
    state = initial_state.copy()
    for i in range(N):
        control = control_trajectory[i]
        state = rk4_step(f, state, control, params, dt)
        # normalize quaternion
        state[6:10] /= np.linalg.norm(state[6:10])
        traj.append(state.copy())
    return np.array(traj)
