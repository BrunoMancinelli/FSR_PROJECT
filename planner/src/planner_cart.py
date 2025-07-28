import numpy as np
import pandas as pd
from scipy.interpolate import interp1d
from scipy.interpolate import CubicSpline

def generate_trajectory(T=7.0, l=1.0, v_max=0.5, omega_max=2, phi_max=np.pi/2-0.01):
    # initial and final states
    xi, yi, thetai = 0.0, 0.0, 0.0
    xf, yf, thetaf = 6.0, 3.5, 1.5
    k = 10

    # s Parameter
    N = 1000
    s = np.linspace(0, 1, N)
    ds = s[1] - s[0]

    # Cubic polynomial coefficients
    alphax = k*np.cos(thetaf) - 3*xf
    betax  = k*np.cos(thetai) - 3*xi
    alphay = k*np.sin(thetaf) - 3*yf
    betay  = k*np.sin(thetai) + 3*yi

    # Flat outputs
    x = s**3*xf + alphax*s**2*(s-1) + betax*s*(s-1)**2
    y = s**3*yf + alphay*s**2*(s-1) + betay*s*(s-1)**2

    # Derivates with respect to s
    dx_ds = np.gradient(x, ds)
    dy_ds = np.gradient(y, ds)
    d2x_ds2 = np.gradient(dx_ds, ds)
    d2y_ds2 = np.gradient(dy_ds, ds)

    theta_s = np.arctan2(dy_ds, dx_ds)
    dtheta_ds = np.gradient(theta_s, ds)
    v_s = np.sqrt(dx_ds**2 + dy_ds**2)
    phi_s = np.arctan2(l * dtheta_ds , v_s)
    dphi_ds = np.gradient(phi_s,ds)
    omega_s = dphi_ds

    # temporal parametrization
    t = np.linspace(0, T, N)
    tau = t / T
    s_t = 3*tau**2 - 2*tau**3
    ds_dt = (6*tau - 6*tau**2) / T

    flag = True
    i = 0
    while flag == True:
        # interpolate on s
        v_s_interp = interp1d(s, v_s, kind='cubic', fill_value="extrapolate")
        dtheta_ds_interp = interp1d(s, dtheta_ds, kind='cubic', fill_value="extrapolate")
        theta_interp = interp1d(s, theta_s, kind='cubic', fill_value="extrapolate")
        phi_interp = interp1d(s, phi_s, kind='cubic', fill_value="extrapolate")
        dphi_dt_interp = interp1d(s, dphi_ds, kind='cubic', fill_value="extrapolate")
        omega_interp = interp1d(s, omega_s, kind='cubic', fill_value="extrapolate")

        v = v_s_interp(s_t) * ds_dt
        omega = omega_interp(s_t) * ds_dt 
        dtheta_dt = dtheta_ds_interp(s_t)*ds_dt
        theta = theta_interp(s_t) 
        phi = phi_interp(s_t)
        dphi_dt = dphi_dt_interp(s_t)*ds_dt
                 
        vmax = np.max(np.abs(v))
        omegamax = np.max(np.abs(omega))
        phimax = np.max(np.abs(phi))
        
        # scaling if the constrains are not satisfied
        if vmax > v_max or omegamax > omega_max or phimax > phi_max:
           scale = max(vmax/v_max , omegamax/omega_max, phimax/phi_max)
           T = T*scale
           t = np.linspace(0,T,N)
           tau = t/T
           
           s_t = 3*tau**2 - 2*tau**3
           ds_dt = (6*tau - 6*tau**2) / T
        else:
           flag = False    
           
    return t, x, y, theta, v, omega, phi

# Trajectory generation
t, x, y, theta, v, omega, phi = generate_trajectory()

# "hold" parameters
t_hold = 5.0  # you can choose to enlarge the trajectory duration to see if the controller converges
N_hold = int(np.round(t_hold / (t[1] - t[0])))
# New extended time sequence
t_extra = np.linspace(t[-1] + (t[1] - t[0]), t[-1] + t_hold, N_hold)
t_new = np.concatenate([t, t_extra])

def extend_signal(sig):
    return np.concatenate([sig, np.full(N_hold, sig[-1])])

x = extend_signal(x)
y = extend_signal(y)
theta = extend_signal(theta)
v = extend_signal(v)
omega = extend_signal(omega)
phi = extend_signal(phi)

# compute coordinate derivatives for full state feedback linearization controller
from scipy.interpolate import CubicSpline

# Interpolation on the new time
cs_x = CubicSpline(t_new, x)
cs_y = CubicSpline(t_new, y)

dot_xd = cs_x.derivative(1)(t_new)
ddot_xd = cs_x.derivative(2)(t_new)
dddot_xd = cs_x.derivative(3)(t_new)

dot_yd = cs_y.derivative(1)(t_new)
ddot_yd = cs_y.derivative(2)(t_new)
dddot_yd = cs_y.derivative(3)(t_new)

# Low-pass filter
def lowpass_alpha_filter(signal, alpha):
    filtered = np.zeros_like(signal)
    filtered[0] = signal[0]
    for i in range(1, len(signal)):
        filtered[i] = alpha * signal[i] + (1 - alpha) * filtered[i - 1]
    return filtered

alpha1 = 0.1
alpha2 = 0.5
alpha3 = 0.7

dot_xd_filt = lowpass_alpha_filter(dot_xd, alpha3)
ddot_xd_filt = lowpass_alpha_filter(ddot_xd, alpha2)
dddot_xd_filt = lowpass_alpha_filter(dddot_xd, alpha1)

dot_yd_filt = lowpass_alpha_filter(dot_yd, alpha3)
ddot_yd_filt = lowpass_alpha_filter(ddot_yd, alpha2)
dddot_yd_filt = lowpass_alpha_filter(dddot_yd, alpha1)

# save the extended Data Frame
df = pd.DataFrame({
    "t": t_new,
    "x": x,
    "y": y,
    "theta": theta,
    "v": v,
    "omega" : omega,
    "phi": phi,
    "dot_xd": dot_xd_filt,
    "ddot_xd": ddot_xd_filt,
    "dddot_xd": dddot_xd_filt,
    "dot_yd": dot_yd_filt,
    "ddot_yd": ddot_yd_filt,
    "dddot_yd": dddot_yd_filt   
})

df.to_csv("trajectory_cart.csv", index=False)
