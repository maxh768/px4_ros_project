from derivation import dynamics
from scipy.integrate import solve_ivp
import numpy as np
import niceplots
from matplotlib import pyplot as plt
import os

mass = 2.0 + (0.016076923076923075 * 4)
g = 9.8
Ixx = 0.023839
Iyy = 0.023942
Izz = 0.044000
kT = 8.54858e-06
kM = 0.016
kd = 8.06428e-05
rotor_params = [( 0.174,  0.174, -0.06, +1),   # 0 front-right, ccw
          (-0.174, -0.174, -0.06, +1),   # 1 back-left,   ccw
          ( 0.174, -0.174, -0.06, -1),   # 2 front-left,  cw
          (-0.174,  0.174, -0.06, -1)]   # 3 back-right,  cw
# rotor_params = [( 0.13,  0.22, 0.0, +1),   # 0 front-right, ccw
#                 (-0.13, -0.20, 0.0, +1),   # 1 back-left,   ccw
#                 ( 0.13, -0.22, 0.0, -1),   # 2 front-left,  cw
#                 (-0.13,  0.20, 0.0, -1)]   # 3 back-right,  cw


# params = [m, g_use, Ixx, Iyy, Izz, kT, kM, kd] + [s for i in range(4) for s in (rx[i], ry[i], rz[i], sg[i])]
params = np.array([mass, g, Ixx, Iyy, Izz, kT, kM, kd] +
                  [v for r in rotor_params for v in r], dtype=float)

def model(t, x, u_func):
    u = u_func(t)
    dx = dynamics(x, u, params).ravel()
    return dx

def make_u_t(time_vec, u_series):
    def u_t(t):
        i = np.clip(np.searchsorted(time_vec, t, side='right') - 1, 0, len(time_vec) - 1)
        return u_series[i]
    return u_t

def sim_model(time_vec, x0, u_vec):
    u_func = make_u_t(time_vec, u_vec)
    sol = solve_ivp(model, (time_vec[0], time_vec[-1]), x0, 'RK45', t_eval = time_vec, args=[u_func], max_step=time_vec[1] - time_vec[0])
    return sol.y


STATE = {'x':  (0,  'x [m]'),      'y':  (1,  'y [m]'),      'z':  (2,  'z [m]'),
         'dx': (3,  'vx [m/s]'),   'dy': (4,  'vy [m/s]'),   'dz': (5,  'vz [m/s]'),
         'q0': (6,  'q0 [-]'),     'q1': (7,  'q1 [-]'),
         'q2': (8,  'q2 [-]'),     'q3': (9,  'q3 [-]'),
         'wx': (10, 'p [rad/s]'),  'wy': (11, 'q [rad/s]'),  'wz': (12, 'r [rad/s]')}

DEFAULT_VARS = ('x', 'y', 'z', 'dx', 'dy', 'dz')

def plot_sim(time_vec, sol, vars=DEFAULT_VARS, path='docs/figures/sim.pdf'):
    unknown = [v for v in vars if v not in STATE]
    if unknown:
        raise ValueError(f'unknown state(s): {unknown}; choose from {list(STATE)}')

    n = len(vars)
    fig, axes = plt.subplots(n, 1, sharex=True, figsize=(11, 2.6*n), layout='constrained')
    axes = [axes] if n == 1 else list(axes)

    for ax, name in zip(axes, vars):
        idx, ylabel = STATE[name]
        ax.plot(time_vec, sol[idx], linewidth=2.0)
        ax.set_ylabel(ylabel, rotation=0, ha='right', va='center')
        ax.grid(False)
        spines = ['left', 'bottom'] if ax is axes[-1] else ['left']
        niceplots.adjust_spines(ax, spines=spines)

    axes[-1].set_xlabel('time [s]')
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    fig.savefig(path, format='pdf', bbox_inches='tight')
    plt.close(fig)

    

if __name__ == "__main__":

    # initial condition
    p0 = np.array([0, 0, -5])
    v0 = np.array([0, 0, 0])
    q0 = np.array([1, 0, 0, 0])
    w0 = np.array([0, 0, 0])

    x0 = np.hstack([p0, v0, q0, w0])

    hz = 50
    dt = 1 / hz

    time_vector = np.arange(0, 30, 1/hz)
    hover_thrust = mass * g / 4
    u_1_step = [hover_thrust] * 4

    u_series = np.tile(u_1_step, (len(time_vector), 1)) # (N, 4)

    y = sim_model(time_vector, x0, u_series)

    plot_sim(time_vector, y)




    