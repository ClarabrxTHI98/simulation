import numpy as np
import matplotlib.pyplot as plt
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
from dynamic_mm import dynamic_system_mm_4dof
from integrator import simulate
# ------------------------------------------------------------
# Simulation Parameters
# ------------------------------------------------------------
dt = config.dt
duration = config.duration
N = int(duration / dt)
g = config.g
m_tot = config.m_tot_mm

# ------------------------------------------------------------
# Initial State
# y = [z, dz, x, dx, theta, dtheta, ll, dll, lr, dlr, eps, deps, phi, dphi]
# ------------------------------------------------------------
state0 = np.zeros(14)
state0[0] = 0.0   # z
state0[2] = 0.0   # x
state0[4] = 0.0   # theta
state0[10] = 0.0   # eps
state0[12] = 0.0   # phi




# ------------------------------------------------------------
# Open-loop control: constant thrust + fixed moving-mass positions
# ------------------------------------------------------------
desired_rotor_speed = math.sqrt(m_tot*g /config.k_T)
omega_r = desired_rotor_speed
T_hover = config.k_T * omega_r**2  # constant thrust

# Specify the target positions for moving masses (meters along beams)
ll_target = 0.3 # right mass
lr_target = 0.3 # right mass

control_traj = []
for i in range(N):
    control_traj.append({
        'T': T_hover,
        'll': ll_target,
        'lr': lr_target
    })

# ------------------------------------------------------------
# Run Simulation
# ------------------------------------------------------------
traj = simulate(dynamic_system_mm_4dof, state0, control_traj, params={}, dt=dt)

# Extract main states
t = np.arange(N) * dt
z_vals = traj[:, 0]      # vertical position
x_vals = traj[:, 2]      # horizontal position
theta_vals = traj[:, 4]  # body pitch angle
ll_vals = traj[:, 6]  # ll
lr_vals = traj[:, 8]  # lr

# ------------------------------------------------------------
# Plot simulation results
# ------------------------------------------------------------
plt.figure()
plt.plot(t, z_vals)
plt.title("Vertical Motion (Z)")
plt.xlabel("Time [s]")
plt.ylabel("Z [m]")
plt.grid(True)
plt.show(block=False)
input("Press Enter to continue...")
plt.close()

plt.figure()
plt.plot(t, x_vals)
plt.title("Horizontal Motion (X)")
plt.xlabel("Time [s]")
plt.ylabel("X [m]")
plt.grid(True)
plt.show(block=False)
input("Press Enter to continue...")
plt.close()

plt.figure()
plt.plot(t, theta_vals)
plt.title("Body Pitch (Theta)")
plt.xlabel("Time [s]")
plt.ylabel("Theta [rad]")
plt.grid(True)
plt.show(block=False)
input("Press Enter to continue to 3D visualization...")
plt.close()

plt.figure()
plt.plot(t, ll_vals)
plt.title("ll")
plt.xlabel("Time [s]")
plt.ylabel("ll [m]")
plt.grid(True)
plt.show(block=False)
input("Press Enter to continue to 3D visualization...")
plt.close()

plt.figure()
plt.plot(t, lr_vals)
plt.title("lr")
plt.xlabel("Time [s]")
plt.ylabel("lr [m]")
plt.grid(True)
plt.show(block=False)
input("Press Enter to continue to 3D visualization...")
plt.close()


# ------------------------------------------------------------
# PyBullet visualization
# ------------------------------------------------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -g)
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

# Fix rotation to align model upright
orientation_fix = p.getQuaternionFromEuler([math.pi / 2, 0, 0])

body_id = p.createMultiBody(
    baseMass=m_tot,
    baseVisualShapeIndex=visual,
    basePosition=[0, 0, z_vals[0]],
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
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ]

# ------------------------------------------------------------
# Animate simulation
# ------------------------------------------------------------
print("Starting 3D animation...")

for i in range(0, N, 5):
    z = z_vals[i]
    x = x_vals[i]
    theta = theta_vals[i]

    # Position
    pos = [x, 0, z]

    # Orientation: pitch about Y-axis
    quat_pitch = p.getQuaternionFromEuler([0, theta, 0])

    # Combine with model orientation fix
    quat_total = quat_multiply(quat_pitch, orientation_fix)

    p.resetBasePositionAndOrientation(body_id, pos, quat_total)
    time.sleep(dt * 5)  # adjust speed

print("Simulation finished. Press Ctrl+C to close PyBullet.")
while True:
    time.sleep(1)
