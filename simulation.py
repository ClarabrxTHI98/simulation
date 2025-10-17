import numpy as np
import matplotlib.pyplot as plt
import config
from dynamic import dynamics
from integrator import simulate

# Parameters
params = {
    'm': config.m,
    'I': np.diag([config.Ixx, config.Iyy, config.Izz]),
    'g': config.g
}

dt = config.dt
duration = config.duration
N = int(duration / dt)

# Initial state
state0 = np.zeros(13)
state0[6] = 1.0  # identity quaternion

# Control
T_hover = params['m'] * params['g']
tau = np.zeros(3)
control = {'T': T_hover, 'tau': tau}
control_traj = [control] * N

# Run simulation
traj = simulate(dynamics, state0, control_traj, params, dt)

# Plot altitude
zs = traj[:, 2]
plt.plot(np.arange(N)*dt, zs)
plt.xlabel("Time [s]")
plt.ylabel("Altitude z [m]")
plt.grid(True)
plt.show()
