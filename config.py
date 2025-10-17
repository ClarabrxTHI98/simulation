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
T = (m*g) #force to compensate

rho= 1.225 #Air density
ks = 0.05 #Coeffient air velocity and rotor speed
Ar = 0.27 #Area internal channel
d = 0.15 #Internal channel radius + 1/2 blade length
r = 0.1 #Distance between the pressure center (where the lift is applied) and the center of the internal channel for yaw compensator airfoils

wind_enabled = False
wind_velocity = [0.0, 0.0, 0.0]

# Simulation
dt = 0.01
duration = 10.0
integrator = "rk4"