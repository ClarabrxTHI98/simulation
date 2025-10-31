import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pathlib import Path
from scipy.optimize import NonlinearConstraint, minimize

from vane_calc_lib import (
    init_state_array, load_polar_csv_to_dataarray, u_from_rpm, rpm_from_u,
    make_torque_eq, neg_total_push, set_alpha_all, sum_force_push, sum_torque, peak_f_raw
)

# Path to your airfoil coefficient file
path_to_coeff_default = Path(__file__).parent.resolve() / "airfoil_data" / "eppler_473_NCrit_5.csv"


# -------------------------------------------------------------------
# Helper: find max force for a given torque using nonlinear constraint
# -------------------------------------------------------------------
def get_max_f_at_given_t(n, r, A, u, beta, max_aoa, T_target, coeff_array):
    state_array = init_state_array(n=n, r=r, A=A, u=u, beta=beta)
    N = state_array.sizes["n"]

    # initial guess and bounds for angle of attack
    x0 = np.zeros(N)
    bounds = [(-max_aoa, max_aoa)] * N

    # equality constraint (torque = T_target)
    torque_eq = make_torque_eq(state_array, coeff_array, T_target)
    nlc = NonlinearConstraint(torque_eq, 0.0, 0.0)

    res = minimize(
        neg_total_push, x0,
        args=(state_array, coeff_array),
        method="trust-constr",
        bounds=bounds,
        constraints=[nlc],
        options={"gtol": 1e-9, "xtol": 1e-9, "verbose": 0},
    )

    alphas_opt = res.x
    set_alpha_all(state_array, coeff_array, alphas_opt)

    f_new = sum_force_push(state_array)
    torque_new = sum_torque(state_array)
    alpha_mean = np.mean(alphas_opt)

    return torque_new, f_new, alpha_mean


# -------------------------------------------------------------------
# Main mapping computation
# -------------------------------------------------------------------
def f_ses_to_torque(
    n: int = 4,
    r: float = 0.21,
    A: float = (0.02 * 0.03),
    u: float = 25,
    path_to_coeff: str = path_to_coeff_default,
    save_csv: bool = True,
    plot_results: bool = True
):
    print("\n🔧 Computing vane force–torque–alpha mapping...")
    coeff_array = load_polar_csv_to_dataarray(path=path_to_coeff)
    alpha_peak_f = peak_f_raw(coeff_array=coeff_array)

    rpm = rpm_from_u(r_max=0.132, cl=0.5, u=u, c=0.02, n=n)
    _, beta = u_from_rpm(r_max=0.132, r_min=0.034, cl=0.5, cd=0.01, rpm=rpm, c=0.02, n=n)

    temp_array = init_state_array(n=n, r=r, A=A, u=u, beta=beta)
    set_alpha_all(temp_array, coeff_array, alpha_peak_f)
    peak_torque = sum_torque(temp_array)

    T_targets = np.linspace(0, peak_torque, 15)
    F_results = np.zeros_like(T_targets)
    Alpha_results = np.zeros_like(T_targets)

    for i, T in enumerate(T_targets):
        T_act, F_act, alpha_mean = get_max_f_at_given_t(
            n=n, r=r, A=A, u=u, beta=beta, max_aoa=alpha_peak_f,
            T_target=T, coeff_array=coeff_array
        )
        F_results[i] = F_act
        Alpha_results[i] = alpha_mean
        print(f"[{i+1:02d}/{len(T_targets)}] Torque={T_act:.4f} Nm | Force={F_act:.4f} N | Alpha={np.degrees(alpha_mean):.2f}°")

    if plot_results:
        # ---- Plot 1: Force vs Torque ----
        plt.figure(figsize=(7, 5))
        plt.plot(T_targets, F_results, "o-", label="Force vs Torque")
        plt.xlabel("Torque [Nm]")
        plt.ylabel("Force [N]")
        plt.title(f"Force–Torque Relationship (u={u:.1f} m/s, n={n})")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        # ---- Plot 2: Alpha vs Torque ----
        plt.figure(figsize=(7, 5))
        plt.plot(T_targets, np.degrees(Alpha_results), "s--", color="tab:red", label="Mean α vs Torque")
        plt.xlabel("Torque [Nm]")
        plt.ylabel("Mean α [deg]")
        plt.title("Mean Angle of Attack vs Torque")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show(block=False)

    if save_csv:
        df = pd.DataFrame({
            "Torque_Nm": T_targets,
            "Force_N": F_results,
            "Alpha_rad": Alpha_results,
            "Alpha_deg": np.degrees(Alpha_results)
        })
        
        csv_path = Path(__file__).parent / "vane_force_torque_map.csv"
        
        # 🪶 DEBUG prints to confirm path and access
        print("\n💾 [DEBUG] Preparing to save CSV...")
        print(f"   - Current working directory: {Path.cwd()}")
        print(f"   - Script directory (where __file__ points): {Path(__file__).parent.resolve()}")
        print(f"   - Intended save path: {csv_path.resolve()}")

        try:
            df.to_csv(csv_path, index=False)
            print(f"✅ CSV successfully saved at: {csv_path.resolve()}")
        except Exception as e:
            print(f"❌ [ERROR] Could not save CSV to {csv_path.resolve()}")
            print(f"   Exception: {e}")

        print("\n📊 Preview of saved data:")
        print(df.head())


    return T_targets, F_results, Alpha_results


# -------------------------------------------------------------------
# Run standalone
# -------------------------------------------------------------------
if __name__ == "__main__":
    f_ses_to_torque(
        n=4,
        r=0.21,
        A=0.02 * 0.03,
        u=25,
        save_csv=True,
        plot_results=True
    )
