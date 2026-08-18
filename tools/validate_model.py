#!/usr/bin/env python3
"""Validate the derived equations of motion against flight data.

Uses the *equation-error* method (Klein & Morelli, Aircraft System
Identification): evaluate f(x, u) at every measured sample and compare it
against the derivative of the measured state. No integration, so there is no
window length to choose and no integration error folded into the result, and
every sample is an independent test.

    tools/validate_model.py                 # every case in data/
    tools/validate_model.py --case hover

Only the six dynamic states are reported. p_dot = v holds by construction, and
q_dot follows from omega, so neither one tests the forces or the moments.
"""

import argparse
import glob
import os

import matplotlib.pyplot as plt
import niceplots
import numpy as np

from derivation import dynamics
from model_sim import params, sim_model

plt.style.use(niceplots.get_style())

FIG_DIR = 'docs/figures'
GRAVITY = 9.8

# (index into the 13-state, label) for the states the model actually predicts
DYNAMIC = [(3, 'dvx [m/s2]'), (4, 'dvy [m/s2]'), (5, 'dvz [m/s2]'),
           (10, 'dwx [rad/s2]'), (11, 'dwy [rad/s2]'), (12, 'dwz [rad/s2]')]

MAX_RATE = 10.0     # rad/s -- gz groundtruth emits ~300 rad/s spikes at quaternion wraps
MIN_HEIGHT = 0.5    # m above the origin before the vehicle counts as airborne


def model_derivative(X, U):
    """f(x, u) at every sample. Returns (N, 13)."""
    return np.array([np.asarray(dynamics(X[k], U[k], params)).ravel()
                     for k in range(len(X))])


def measured_derivative(t, X):
    """d/dt of the measured state by central difference. Returns (N, 13)."""
    return np.gradient(X, t, axis=0)


def valid_mask(X, erode=3):
    """Samples where the comparison is meaningful.

    Excludes time on the ground -- the ground supplies a normal force the model
    has no term for, so it predicts falling while the vehicle sits still -- and
    the groundtruth angular-rate spikes. Eroded at the edges, because a central
    difference at a boundary reaches into the excluded region.
    """
    m = np.all(np.abs(X[:, 10:13]) < MAX_RATE, axis=1) & (X[:, 2] < -MIN_HEIGHT)
    for _ in range(erode):
        m &= np.roll(m, 1) & np.roll(m, -1)
    return m


def residuals(case):
    """(t, model, measured, mask) for one loaded .npz."""
    t, X, U = case['t'], case['X'], case['U']
    return t, model_derivative(X, U), measured_derivative(t, X), valid_mask(X)


def summarise(name, t, F, D, mask):
    """One row per state: RMS residual, RMS signal, and the ratio."""
    print(f'\n{name}   {mask.sum()} of {len(t)} samples used '
          f'({100*mask.mean():.0f}% airborne and sane)')
    print('  state          RMS residual     RMS signal      ratio')
    for i, label in DYNAMIC:
        r = F[mask, i] - D[mask, i]
        rms_r = float(np.sqrt(np.mean(r ** 2)))
        rms_s = float(np.sqrt(np.mean(D[mask, i] ** 2)))
        print(f'  {label:14} {rms_r:12.4f} {rms_s:14.4f} {rms_r/max(rms_s,1e-12):10.3f}')
    acc = float(np.sqrt(np.mean((F[mask, 3:6] - D[mask, 3:6]) ** 2)))
    print(f'  translational residual {acc:.4f} m/s2 = {100*acc/GRAVITY:.2f}% of g')


def plot_residuals(name, t, F, D, mask, path):
    """Model against measured derivative. Left column forces, right moments."""
    tm = t[mask]
    fig, axes = plt.subplots(3, 2, figsize=(13, 9), layout='constrained', sharex=True)

    for k, (i, label) in enumerate(DYNAMIC):
        ax = axes[k % 3, k // 3]
        # measured wide and pale behind, model thin and opaque on top
        ax.plot(tm, D[mask, i], linewidth=5.0, alpha=0.30, zorder=1, label='measured')
        ax.plot(tm, F[mask, i], linewidth=1.6, alpha=1.00, zorder=2, label='model')
        ax.set_ylabel(label, rotation=0, ha='right', va='center')
        ax.grid(False)
        if k == 0:
            ax.legend(loc='lower left', bbox_to_anchor=(0.0, 1.0), ncol=2, frameon=False)
        bottom = (k % 3) == 2
        niceplots.adjust_spines(ax, spines=['left', 'bottom'] if bottom else ['left'])
        if bottom:
            ax.set_xlabel('time [s]')

    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    fig.savefig(path, format='pdf', bbox_inches='tight')
    plt.close(fig)
    print(f'  wrote {path}')


# --------------------------------------------------------------- horizon sweep

HORIZONS = (0.5, 1.0, 2.0, 5.0, 10.0, 20.0)


def attitude_error_deg(q_model, q_truth):
    """Angle between two attitudes, handling the q/-q double cover."""
    d = abs(float(np.dot(q_model, q_truth)) /
            (np.linalg.norm(q_model) * np.linalg.norm(q_truth)))
    return float(np.degrees(2.0 * np.arccos(min(d, 1.0))))


def horizon_sweep(case, horizons=HORIZONS):
    """Free-run the model over windows of each length, from truth each time.

    This is k-step-ahead prediction: the standard way to score a nonlinear model
    without asking it to do something no model can. A quadrotor is a double
    integrator with no restoring force, so any acceleration residual becomes
    0.5*a*t^2 in position -- a perfect model diverges too, just more slowly.
    """
    t, X, U = case['t'], case['X'], case['U']
    airborne = X[:, 2] < -MIN_HEIGHT

    pos, att, count = [], [], []
    for H in horizons:
        pe, ae = [], []
        for t_start in np.arange(t[0], t[-1] - H, max(H / 3.0, 0.5)):
            m = (t >= t_start) & (t < t_start + H)
            if m.sum() < 10 or not airborne[m].all():
                continue                       # never straddle ground contact
            sol = sim_model(t[m], X[m][0], U[m])
            pe.append(np.linalg.norm(sol[:3, -1] - X[m][-1, :3]))
            ae.append(attitude_error_deg(sol[6:10, -1], X[m][-1, 6:10]))
        pos.append(np.sqrt(np.mean(np.square(pe))) if pe else np.nan)
        att.append(np.sqrt(np.mean(np.square(ae))) if ae else np.nan)
        count.append(len(pe))
    return np.array(pos), np.array(att), np.array(count)


def plot_horizon_sweep(results, path, horizons=HORIZONS):
    """RMS error against prediction horizon, log-log, one line per maneuver."""
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), layout='constrained')
    h = np.asarray(horizons, float)

    for k, (ax, key, ylabel) in enumerate(
            zip(axes, (0, 1), ('position\nRMS [m]', 'attitude\nRMS [deg]'))):
        for name, res in sorted(results.items()):
            ax.loglog(h, res[key], linewidth=2.0, label=name)
        # t^2 reference: what a constant acceleration bias would give
        ref = res[key][1] * (h / h[1]) ** 2
        ax.loglog(h, ref, linestyle='--', linewidth=1.5, color='#999999',
                  zorder=0, label='$t^2$ reference')
        ax.set_ylabel(ylabel, rotation=0, ha='right', va='center')
        ax.set_xlabel('prediction horizon [s]')
        ax.grid(False)
        if k == 0:
            ax.legend(loc='lower left', bbox_to_anchor=(0.0, 1.0), ncol=4,
                      frameon=False, fontsize='small')
        niceplots.adjust_spines(ax, spines=['left', 'bottom'])

    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    fig.savefig(path, format='pdf', bbox_inches='tight')
    plt.close(fig)
    print(f'\nwrote {path}')


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--case', help='single case name, e.g. hover (default: all)')
    p.add_argument('--data-dir', default='data')
    p.add_argument('--fig-dir', default=FIG_DIR)
    p.add_argument('--no-sweep', action='store_true',
                   help='skip the horizon sweep, which is the slow part')
    args = p.parse_args()

    files = ([os.path.join(args.data_dir, args.case + '.npz')] if args.case
             else sorted(glob.glob(os.path.join(args.data_dir, '*.npz'))))
    if not files:
        p.error(f'no .npz files in {args.data_dir}')

    sweeps = {}
    for f in files:
        name = os.path.splitext(os.path.basename(f))[0]
        case = np.load(f)

        t, F, D, mask = residuals(case)
        summarise(name, t, F, D, mask)
        plot_residuals(name, t, F, D, mask,
                       os.path.join(args.fig_dir, f'residual_{name}.pdf'))

        if not args.no_sweep:
            pos, att, count = horizon_sweep(case)
            sweeps[name] = (pos, att)
            print('  horizon [s] ' + ' '.join(f'{h:8.1f}' for h in HORIZONS))
            print('  pos err [m] ' + ' '.join(f'{v:8.3f}' for v in pos))
            print('  windows     ' + ' '.join(f'{c:8d}' for c in count))

    if sweeps:
        plot_horizon_sweep(sweeps, os.path.join(args.fig_dir, 'horizon_sweep.pdf'))


if __name__ == '__main__':
    main()
