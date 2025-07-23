import numpy as np
import pandas as pd
from scipy.interpolate import interp1d
from scipy.interpolate import CubicSpline
import numpy as np
from scipy.interpolate import interp1d

import numpy as np
from scipy.interpolate import interp1d

def generate_trajectory(T=8, l=1.0, v_max=0.5, omega_max=2, phi_max=np.pi/2-0.01):
    # Initial and final angular values
    theta_i = 0.0
    theta_f = np.pi  # 90 gradi
    dtheta = theta_f - theta_i

    # s parametrization
    N = 1000
    s = np.linspace(0, 1, N)
    ds = s[1] - s[0]

    # Smooth angular evolution (cubic polynomial)
    theta_s = theta_i + 3*dtheta*s**2 - 2*dtheta*s**3

    # Circle parameterized by theta_s
    R = 2
    x = R * np.sin(theta_s)
    y = R * (1 - np.cos(theta_s))

    # Derivates with respect to s
    dx_ds = np.gradient(x, ds)
    dy_ds = np.gradient(y, ds)
    d2x_ds2 = np.gradient(dx_ds, ds)
    d2y_ds2 = np.gradient(dy_ds, ds)

    theta_s_val = np.arctan2(dy_ds, dx_ds)
    dtheta_ds = np.gradient(theta_s_val, ds)
    v_s = np.sqrt(dx_ds**2 + dy_ds**2)
    phi_s = np.arctan2(l * dtheta_ds, v_s)
    dphi_ds = np.gradient(phi_s, ds)
    omega_s = dphi_ds

    # Temporal parametrization
    t = np.linspace(0, T, N)
    tau = t / T
    s_t = 3 * tau**2 - 2 * tau**3
    ds_dt = (6 * tau - 6 * tau**2) / T

    flag = True
    while flag:
        # Interpolation
        v_s_interp = interp1d(s, v_s, kind='cubic', fill_value="extrapolate")
        dtheta_ds_interp = interp1d(s, dtheta_ds, kind='cubic', fill_value="extrapolate")
        theta_interp = interp1d(s, theta_s_val, kind='cubic', fill_value="extrapolate")
        phi_interp = interp1d(s, phi_s, kind='cubic', fill_value="extrapolate")
        dphi_dt_interp = interp1d(s, dphi_ds, kind='cubic', fill_value="extrapolate")
        omega_interp = interp1d(s, omega_s, kind='cubic', fill_value="extrapolate")

        v = v_s_interp(s_t) * ds_dt
        omega = omega_interp(s_t) * ds_dt
        dtheta_dt = dtheta_ds_interp(s_t) * ds_dt
        theta = theta_interp(s_t)
        phi = phi_interp(s_t)
        dphi_dt = dphi_dt_interp(s_t) * ds_dt

        vmax = np.max(np.abs(v))
        omegamax = np.max(np.abs(omega))
        phimax = np.max(np.abs(phi))
	
	#time scaling
        if vmax > v_max or omegamax > omega_max or phimax >= phi_max:
            scale = max(vmax / v_max, omegamax / omega_max, phimax / phi_max)
            T = T * scale
            t = np.linspace(0, T, N)
            tau = t / T
            s_t = 3 * tau**2 - 2 * tau**3
            ds_dt = (6 * tau - 6 * tau**2) / T
        else:
            flag = False

    return t, x, y, theta, v, omega, phi

#Trajectory generation
t, x, y, theta, v, omega, phi = generate_trajectory()

# compute coordinate derivatives for full state feedback linearization controller

# Define a simple low-pass filter function (exponential moving average)
def lowpass_alpha_filter(signal, alpha):
    filtered = np.zeros_like(signal)
    filtered[0] = signal[0]  # inizializzazione
    for i in range(1, len(signal)):
    	# apply the recursive formula for exponential smoothing
        filtered[i] = alpha * signal[i] + (1 - alpha) * filtered[i - 1]
    return filtered

# Compute trajectory derivatives using cubic spline interpolation
cs_x = CubicSpline(t, x)
cs_y = CubicSpline(t, y)

dot_xd = cs_x.derivative(1)(t)
ddot_xd = cs_x.derivative(2)(t)
dddot_xd = cs_x.derivative(3)(t)

dot_yd = cs_y.derivative(1)(t)
ddot_yd = cs_y.derivative(2)(t)
dddot_yd = cs_y.derivative(3)(t)

# Apply low-pass filters with different smoothing factors (alpha)
alpha1 = 0.1;  # lower alpha = smoother signal + more delay
alpha2 = 0.5;
alpha3 = 0.7;

dot_xd_filt = lowpass_alpha_filter(dot_xd, alpha3)
ddot_xd_filt = lowpass_alpha_filter(ddot_xd, alpha2)
dddot_xd_filt = lowpass_alpha_filter(dddot_xd, alpha1)

dot_yd_filt = lowpass_alpha_filter(dot_yd, alpha3)
ddot_yd_filt = lowpass_alpha_filter(ddot_yd, alpha2)
dddot_yd_filt = lowpass_alpha_filter(dddot_yd, alpha1)


df = pd.DataFrame({
    "t": t,
    "x": x,
    "y": y,
    "theta": theta,
    "v": v,
    "omega" : omega,
    "phi": phi,
    "dot_xd": dot_xd_filt,
    "ddot_xd ": ddot_xd_filt,
    "dddot_xd": dddot_xd_filt,
    "dot_yd": dot_yd_filt,
    "ddot_yd": ddot_yd_filt,
    "dddot_yd": dddot_yd_filt   
})

df.to_csv("trajectory_circ.csv", index=False)

