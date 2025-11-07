import numpy as np
import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import math
import time
from scipy.spatial.transform import Rotation as R
from control_flap_pid import PIDFlapController
import os
import sys 
from pathlib import Path

# Add the parent directory to Python path
script_dir = Path(__file__).resolve().parent
parent_dir = script_dir.parent  # /home/clara/Documents/vortifer/simulation
sys.path.append(str(parent_dir))

import config
from dynamic_flaps import build_dynamics
from integrator import simulate

# ------------------------------------------------------------
# Simulation Parameters
# ------------------------------------------------------------
dt = config.dt
duration = config.duration
N = int(duration / dt)
g = config.g
m = config.m
r = config.r_tot
h0 = 0.03  # baseline flap height for hover

# ------------------------------------------------------------
# Build Linear Dynamics (A, B, C, D)
# ------------------------------------------------------------
A, B, C, D, sys = build_dynamics()

# ------------------------------------------------------------
# Initial State (12x1)
# [z, dz, yaw, dyaw, x, dx, roll, droll, y, dy, pitch, dpitch]
# ------------------------------------------------------------
state = np.zeros(12)

# ------------------------------------------------------------
# Controller Initialization
vane_csv= str(script_dir / "airfoil_data" / "eppler_473_NCrit_9.csv")

pid_controller = PIDFlapController(
    (0.010, 0.001, 15),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.1, 0.1, 0.2),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    0.0, 0.0, 0.0,
    vane_csv_path=vane_csv
)

# ------------------------------------------------------------
# Storage
# ------------------------------------------------------------
traj = np.zeros((N, 12))
traj[0] = state
pitch_refs = np.zeros(N)
roll_refs = np.zeros(N)
z_refs = np.zeros(N)
tau_yaw_vals = np.zeros(N)
alpha_vane_vals = np.zeros(N)



# ------------------------------------------------------------
# Simulation loop
# ------------------------------------------------------------
t = np.arange(N) * dt

for i in range(1, N):

    # --------------------------------------------------------
    # Time-varying references
    # --------------------------------------------------------

    # Roll reference step
    roll_ref = 0.1 if t[i] < duration / 2 else 0.0
    roll_refs[i] = roll_ref
    

    # Roll reference step
    pitch_ref = 0.1 if t[i] < duration / 2 else 0.0
    pitch_refs[i] = pitch_ref

    # Altitude step reference
    if t[i] < duration / 3:
        z_ref = 0.0
    elif t[i] < 2 * duration / 3:
        z_ref = 10.0
    else:
        z_ref = 0.0
    z_refs[i] = z_ref

    # --------------------------------------------------------
    # Extract current states
    # --------------------------------------------------------
    z, dz = state[0], state[1]
    yaw, dyaw = state[2], state[3]
    roll, droll = state[6], state[7]
    pitch, dpitch = state[10], state[11]

    # --------------------------------------------------------
    # Compute control
    # --------------------------------------------------------
    pid_control = pid_controller.step(
        z=z, pitch=pitch, roll=roll, yaw=yaw,
        z_ref=z_ref, pitch_ref=pitch_ref,
        roll_ref=roll_ref, yaw_ref=0.0
    )

    T = pid_control['T']
    omega = pid_control['omega']
    tau_pitch = pid_control['tau_pitch']
    tau_roll = pid_control['tau_roll']
    tau_yaw = pid_control['tau_yaw']
    alpha_vane = pid_control['alpha_vane']
    tau_yaw_vals[i] = tau_yaw
    alpha_vane_vals[i] = alpha_vane

   
    # --------------------------------------------------------
    # Debug every 0.5s
    # --------------------------------------------------------
    if i % int(0.5 / dt) == 0:
        print(
            f"t={t[i]:.2f}s | z={z:.2f} | z_ref={z_ref:.2f} | pitch_ref={pitch_ref:.2f}\n"
            f"   T={T:.3f} | omega={omega:.3f}  "
            f"tau_yaw={tau_yaw:.5e}\n"
        )

    # --------------------------------------------------------
    # Apply control to linear dynamics
    # --------------------------------------------------------
    u = np.array([T, tau_roll, tau_pitch, tau_yaw])
    state_dot = A @ state + B @ u
    state = state + state_dot * dt

    traj[i] = state

# ------------------------------------------------------------
# Extract signals for plotting
# ------------------------------------------------------------
z_vals = traj[:, 0]
yaw_vals = traj[:, 2]
x_vals = traj[:, 4]
roll_vals = traj[:, 6]
y_vals = traj[:, 8]
pitch_vals = traj[:, 10]

# ------------------------------------------------------------
# Plot Results
# ------------------------------------------------------------
def hold_plot():
    plt.show(block=False)
    input("Press Enter to continue...")
    plt.close()

plt.figure()
plt.plot(t, z_vals, label="Altitude (z)")
plt.plot(t, z_refs, '--', label="z_ref")
plt.title("Altitude Tracking")
plt.xlabel("Time [s]")
plt.ylabel("Z [m]")
plt.legend(); plt.grid(True); hold_plot()

plt.figure()
plt.plot(t, x_vals, label="X")
plt.title("Horizontal Tracking")
plt.xlabel("Time [s]")
plt.ylabel("X [m]")
plt.legend(); plt.grid(True); hold_plot()

plt.figure()
plt.plot(t, y_vals, label="Y")
plt.title("Horizontal Tracking Y")
plt.xlabel("Time [s]")
plt.ylabel("Y [m]")
plt.legend(); plt.grid(True); hold_plot()

plt.figure()
plt.plot(t, tau_yaw_vals, label="Tau Yaw")
plt.title("Yaw Torque Command")
plt.xlabel("Time [s]")
plt.ylabel("τ_yaw [Nm]")
plt.legend()
plt.grid(True)
hold_plot()

plt.figure()
plt.plot(t, alpha_vane_vals, label="Alpha Vane")
plt.title("Vane Angle Over Time")
plt.xlabel("Time [s]")
plt.ylabel("Alpha [rad]")
plt.legend()
plt.grid(True)
hold_plot()


plt.figure()
plt.plot(t, roll_vals, label="Roll")
plt.plot(t, roll_refs, '--', label="Roll Ref")
plt.title("Roll Tracking")
plt.xlabel("Time [s]")
plt.ylabel("Roll [rad]")
plt.legend(); plt.grid(True); hold_plot()

plt.figure()
plt.plot(t, pitch_vals, label="Pitch")
plt.plot(t, pitch_refs, '--', label="Pitch Ref")
plt.title("Pitch Tracking")
plt.xlabel("Time [s]")
plt.ylabel("Pitch [rad]")
plt.legend(); plt.grid(True); hold_plot()

plt.figure()
plt.plot(t, yaw_vals)
plt.title("Yaw Evolution")
plt.xlabel("Time [s]")
plt.ylabel("Yaw [rad]")
plt.grid(True); hold_plot()

# ------------------------------------------------------------
# PyBullet Visualization
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

orientation_fix = p.getQuaternionFromEuler([math.pi/2, 0, 0])
body_id = p.createMultiBody(
    baseMass=m,
    baseVisualShapeIndex=visual,
    basePosition=[0, 0, z_vals[0]],
    baseOrientation=orientation_fix
)

# Before the loop — create the axes once and store their IDs
axis_len = 0.1
x_axis_id = p.addUserDebugLine([0, 0, 0], [axis_len, 0, 0], [1, 0, 0], lineWidth=3)
y_axis_id = p.addUserDebugLine([0, 0, 0], [0, axis_len, 0], [0, 1, 0], lineWidth=3)
z_axis_id = p.addUserDebugLine([0, 0, 0], [0, 0, axis_len], [0, 0, 1], lineWidth=3)

print("Starting 3D animation...")
for i in range(0, N, 6):
    pos = [float(x_vals[i]), float(y_vals[i]), float(z_vals[i])]
    roll, pitch, yaw = float(roll_vals[i]), float(pitch_vals[i]), float(yaw_vals[i])
    quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

    def quat_multiply(q1, q2):
        x1, y1, z1, w1 = q1 
        x2, y2, z2, w2 = q2
        return [
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ]

    q_total = quat_multiply(quat, orientation_fix)
    p.resetBasePositionAndOrientation(body_id, pos, q_total)

    '''

    # --- Camera control (put here) ---
    if i < N / 3:
        # Side view
        p.resetDebugVisualizerCamera(1.0, 90, -20, [0, 0, 0])
    elif i < 2 * N / 3:
        # Top view
        p.resetDebugVisualizerCamera(1.0, 0, -89, [0, 0, 0])
    else:
        # Follow camera (moves with the body)
        p.resetDebugVisualizerCamera(0.5, 0, -30, pos)
    # -------------------------------

    '''

    # Body frame unit vectors in local frame
    x_axis_local = [axis_len, 0, 0]
    y_axis_local = [0, axis_len, 0]
    z_axis_local = [0, 0, axis_len]

    # Transform local axes to world frame using the current body orientation
    x_axis_world, _ = p.multiplyTransforms(pos, q_total, x_axis_local, [0, 0, 0, 1])
    y_axis_world, _ = p.multiplyTransforms(pos, q_total, y_axis_local, [0, 0, 0, 1])
    z_axis_world, _ = p.multiplyTransforms(pos, q_total, z_axis_local, [0, 0, 0, 1])

    # Update the same debug lines
    p.addUserDebugLine(pos, x_axis_world, [1, 0, 0], lineWidth=400, replaceItemUniqueId=x_axis_id)
    p.addUserDebugLine(pos, y_axis_world, [0, 1, 0], lineWidth=400, replaceItemUniqueId=y_axis_id)
    p.addUserDebugLine(pos, z_axis_world, [0, 0, 1], lineWidth=400, replaceItemUniqueId=z_axis_id)

    '''
    # --- Cameras ---
    view_static = p.computeViewMatrixFromYawPitchRoll([0, 0, 0.5], 2, 90, -30, 0, 2) #check better this function doc
    view_follow = p.computeViewMatrixFromYawPitchRoll(pos, 0.3, 0, -20, 0, 2) 
    proj = p.computeProjectionMatrixFOV(60, 1, 0.1, 100) 
    img_static = p.getCameraImage(640, 480, view_static, proj) 
    img_follow = p.getCameraImage(640, 480, view_follow, proj)
    '''

    time.sleep(dt * 10)

print("Simulation finished.")
while True:
    time.sleep(1)
