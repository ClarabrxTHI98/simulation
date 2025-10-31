import numpy as np
import sys
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import pybullet as p
import pybullet_data
import math
import time
import os
import sys 
from pathlib import Path

# Current script folder
script_dir = Path(__file__).parent

# Add the parent directory to Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


import config
from dynamic import dynamics
from integrator import simulate
# ------------------------------------------------------------
# Simulation Parameters
# ------------------------------------------------------------
params = {
    'm': config.m,
    'I': np.diag([config.Ixx, config.Iyy, config.Izz]),
    'g': config.g
}

dt = config.dt
duration = config.duration
N = int(duration / dt)

# ------------------------------------------------------------
# Initial State
# state = [r(3), v(3), q(4), ω(3)]
# ------------------------------------------------------------
state0 = np.zeros(13)
state0[6] = 1.0  # quaternion = [1, 0, 0, 0] (aligned with world frame)

# ------------------------------------------------------------
# Controls (hover)
# ------------------------------------------------------------
desired_rotor_speed = math.sqrt(params['m']*params['g'] /config.k_T)
omega_r = desired_rotor_speed

T = config.k_T * omega_r**2
tau_z = -config.k_Q * omega_r**2
control = {'T': T, 'tau': np.array([0.0, 0.0, tau_z])}
control_traj = [control] * N

# ------------------------------------------------------------
# Run Simulation
# ------------------------------------------------------------
# Run Simulation
traj = simulate(dynamics, state0, control_traj, params, dt)

# ----------------------------------------
# Normalize quaternions in the trajectory
# ----------------------------------------
for i in range(len(traj)):
    q = traj[i, 6:10]
    norm = np.linalg.norm(q)
    if norm == 0:
        print(f"Warning: zero quaternion at step {i}, replacing with [1,0,0,0]")
        traj[i, 6:10] = np.array([1.0, 0.0, 0.0, 0.0])
    else:
        traj[i, 6:10] = q / norm

positions = traj[:, 0:3]
quaternions = traj[:, 6:10]  # [q0, q1, q2, q3]

# ------------------------------------------------------------
# Extract Altitude and Yaw for Plotting
# ------------------------------------------------------------
zs = positions[:, 2]
yaws = [R.from_quat([q[1], q[2], q[3], q[0]]).as_euler('zyx')[0] for q in quaternions]

# ------------------------------------------------------------
# Plot Results
# ------------------------------------------------------------
plt.figure()
plt.plot(np.arange(N) * dt, yaws)
plt.title("Yaw Angle vs Time")
plt.xlabel("Time [s]")
plt.ylabel("Yaw [rad]")
plt.grid(True)
plt.show(block=False)
input("Press Enter to continue...")

plt.figure()
plt.plot(np.arange(N) * dt, zs)
plt.title("Altitude vs Time")
plt.xlabel("Time [s]")
plt.ylabel("Z [m]")
plt.grid(True)
plt.show(block=False)
input("Press Enter to continue to visualization...")
plt.close("all")

# ------------------------------------------------------------
# PyBullet Visualization
# ------------------------------------------------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -params['g'])
p.loadURDF("plane.urdf")

# Load CAD model
# Path to the CAD file relative to this script
script_dir = Path(__file__).resolve().parent
cad_path = (script_dir.parent / "CAD" / "vortifer.obj").resolve()
print("Looking for CAD at:", cad_path)
if not cad_path.exists():
    raise FileNotFoundError(f"Could not find CAD file at {cad_path}")
# Convert Path to string
cad_path_str = str(cad_path)


visual = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName=cad_path_str,
    meshScale=[0.01, 0.01, 0.01]
)

# Fix: Rotate 90 degrees about X to align correctly
orientation_fix = p.getQuaternionFromEuler([math.pi / 2, 0, 0])

body_id = p.createMultiBody(
    baseMass=params['m'],
    baseVisualShapeIndex=visual,
    basePosition=[0, 0, 0.5],
    baseOrientation=orientation_fix
)

p.resetDebugVisualizerCamera(2.0, 45, -30, [0, 0, 0.5])

# ------------------------------------------------------------
# Helper: Quaternion multiplication
# ------------------------------------------------------------
def quat_multiply(q1, q2):
    """Hamilton product q = q1 * q2"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    ]

# ------------------------------------------------------------
# Animate Simulation
# ------------------------------------------------------------
print("Starting animation...")

for i in range(0, N, 5):  # every 5 frames for smoother motion
    pos = positions[i]
    q_sim = quaternions[i]  # [q0, q1, q2, q3]
    # Convert to PyBullet order: [x, y, z, w]
    q_bullet = [q_sim[1], q_sim[2], q_sim[3], q_sim[0]]
    # Combine with model orientation fix
    q_total = quat_multiply(q_bullet, orientation_fix)
    p.resetBasePositionAndOrientation(body_id, pos.tolist(), q_total)
    time.sleep(dt * 5)  # playback speed multiplier

print("Simulation finished. Press Ctrl+C or close the PyBullet window to exit.")
while True:
    time.sleep(1)
