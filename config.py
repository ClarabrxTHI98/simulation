# Parameters
m = 3.0 #mass of the total system
r_t = 0.2 #radius of the toroid
r_m = 0.3 #radius of the pilz (mushroom)
d = 1
Ixx = 0.04 #just a rough estimate
Iyy = 0.04 #just a rough estimate
Izz = 0.06 #just a rough estimate

r_tot = 0.5 #total radius (r_t + r_m for now)
g = 9.81
kf = 1 #check for reasonable coefficients
l= 0.1 #width of the flaps
pi = 3.1415
Ci = 2.0*pi*r_m #circumference pilz
T = 0 # gravity force to compensate

rho= 1.225 #Air density
ks = 0.05 #Coeffient air velocity and rotor speed
Ar = 0.27 #Area internal channel
d_int = 0.15 #Internal channel radius + 1/2 blade length
r = 0.1 #Distance between the pressure center (where the alift is applied) and the center of the internal channel for yaw compensator airfoils
n = 4 # number of flaps in the ninternal channel
k_alpha = 2 #for now, no idea
E = k_alpha*rho*ks*ks/2*Ar*n

wind_enabled = False
wind_velocity = [0.0, 0.0, 0.0]
k_T = 1e-3     # thrust coefficient [N·s²] RANDOM
k_Q = 1e-4      # torque coefficient [N·m·s²] RANDOM

# Simulation
dt = 0.001
duration = 10.0
integrator = "rk4"

### Moving masses parameters
m_l = 0.3
m_r = 0.3
ll_1 = 0.3
lr_1 = 0.3
J_rot = Izz
J_l = 4.5e-4 
J_r = 4.5e-4
d_mm = 0.232
m_tot_mm = m + m_l + m_r