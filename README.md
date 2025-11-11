# Vortifer™ Simulation in Python

This repository contains the source code for the **simulation and control of Vortifer™**, implemented entirely in Python.  
The model includes internal-channel flaps, realistic airfoil-based vane dynamics, and a detailed aerodynamic control architecture.  
All relevant scripts are located inside the `flaps` directory.

---

## Repository Structure

flaps/
 ├── simulation_flaps.py  
 ├── simulation_flaps_ramp.py  
 ├── simulation_flaps_double.py  
 ├── control_flap_pid.py  
 ├── control_flap_pid_ideal.py  
 ├── airfoil_data/  
 └── plots/   (auto-generated)  


- **`flaps/`** — Main simulation code and all controller implementations  
- **`airfoil_data/`** — Real airfoil lookup tables used by the vane/flap controller  
- **`CAD/`** — 3D models used for PyBullet visualization  
- **`plots/`** — Auto-generated plots saved at every simulation run  

---

## Running the Simulations

### **1. Step Altitude Simulation**
Runs Vortifer™ with a **step altitude reference**:
   python simulation_flaps.py

### **2. Ramp Altitude Simulation**

Runs Vortifer with a **ramp altitude trajectory**:
    python simulation_flaps_ramp.py

### **3. Coaxial Rotor Simulation**

Runs the model using a **coaxial rotor configuration**:
    python simulation_flaps_double.py

## Controllers

All control modules are implemented inside the "flaps" directory.

Realistic PID Controller (default)

Used by:
    - simulation_flaps.py
    - simulation_flaps_ramp.py

Import with:
    from control_flap_pid import PIDFlapController

Idealized Controller (optional)

To remove aerodynamic data for the airfoils and use an idealized model:
    from control_flap_pid_ideal import PIDFlapController

## Plot Display and Saving Behavior

Each simulation will:
    - Show plots interactively
    - Wait for you to press Enter before closing each figure
    - Automatically save all plots inside a time-stamped folder located in:
        flaps/plots/
**Note:** Plot saving occurs only if the following lines are uncommented in the simulation script:
   - plot_dir = script_dir / "plots" / timestamp
   - os.makedirs(plot_dir, exist_ok=True)
   - plt.savefig(plot_dir / filename)

## Dependencies

Install required packages running:
pip install -r requirements.txt



