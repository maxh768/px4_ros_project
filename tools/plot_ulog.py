#!/usr/bin/env python3
"""Plot signals from a PX4 .ulg flight log.

Run with no arguments -- from VS Code, or `python3 tools/plot_ulog.py` -- and it
writes both standard figures for the newest flight:

    docs/figures/path.pdf     ground track coloured by time, plus z against time
    docs/figures/states.pdf   position, velocity and body rates in two columns

Every signal is one line in SIGNALS below: a short name mapped to up to three
series -- what was commanded, what the estimator believed, and what was actually
true in simulation. Adding a new signal should never require touching the
plotting code.

    tools/plot_ulog.py                       # both figures, newest log
    tools/plot_ulog.py --list                # what this log contains
    tools/plot_ulog.py p q r                 # a custom set of signals

Field paths are "topic.field"; array members use pyulog's flattened form,
e.g. vehicle_angular_velocity.xyz[0].
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import niceplots
import numpy as np
from matplotlib.collections import LineCollection
from pyulog import ULog

# See guidelines/figures.md — niceplots style, offset spines, horizontal
# y-labels, no grid, boxless legend, PDF output.
plt.style.use(niceplots.get_style())

DEFAULT_LOG = 'flight_logs/latest.ulg'
FIG_DIR = 'docs/figures'

# name: (ylabel, commanded, estimated, truth)   -- commanded/truth may be None
SIGNALS = {
    'x':  ('x [m]',      'trajectory_setpoint.position[0]', 'vehicle_local_position.x',  'vehicle_local_position_groundtruth.x'),
    'y':  ('y [m]',      'trajectory_setpoint.position[1]', 'vehicle_local_position.y',  'vehicle_local_position_groundtruth.y'),
    'z':  ('z [m]',      'trajectory_setpoint.position[2]', 'vehicle_local_position.z',  'vehicle_local_position_groundtruth.z'),
    'vx': ('vx [m/s]',   'trajectory_setpoint.velocity[0]', 'vehicle_local_position.vx', 'vehicle_local_position_groundtruth.vx'),
    'vy': ('vy [m/s]',   'trajectory_setpoint.velocity[1]', 'vehicle_local_position.vy', 'vehicle_local_position_groundtruth.vy'),
    'vz': ('vz [m/s]',   'trajectory_setpoint.velocity[2]', 'vehicle_local_position.vz', 'vehicle_local_position_groundtruth.vz'),

    'qw': ('qw [-]', 'vehicle_attitude_setpoint.q_d[0]', 'vehicle_attitude.q[0]', 'vehicle_attitude_groundtruth.q[0]'),
    'qx': ('qx [-]', 'vehicle_attitude_setpoint.q_d[1]', 'vehicle_attitude.q[1]', 'vehicle_attitude_groundtruth.q[1]'),
    'qy': ('qy [-]', 'vehicle_attitude_setpoint.q_d[2]', 'vehicle_attitude.q[2]', 'vehicle_attitude_groundtruth.q[2]'),
    'qz': ('qz [-]', 'vehicle_attitude_setpoint.q_d[3]', 'vehicle_attitude.q[3]', 'vehicle_attitude_groundtruth.q[3]'),

    'p':  ('p [rad/s]', 'vehicle_rates_setpoint.roll',  'vehicle_angular_velocity.xyz[0]', 'vehicle_angular_velocity_groundtruth.xyz[0]'),
    'q':  ('q [rad/s]', 'vehicle_rates_setpoint.pitch', 'vehicle_angular_velocity.xyz[1]', 'vehicle_angular_velocity_groundtruth.xyz[1]'),
    'r':  ('r [rad/s]', 'vehicle_rates_setpoint.yaw',   'vehicle_angular_velocity.xyz[2]', 'vehicle_angular_velocity_groundtruth.xyz[2]'),

    'thrust_z': ('Tz [-]', None, 'vehicle_thrust_setpoint.xyz[2]', None),
    'torque_x': ('Mx [-]', None, 'vehicle_torque_setpoint.xyz[0]', None),
    'torque_y': ('My [-]', None, 'vehicle_torque_setpoint.xyz[1]', None),
    'torque_z': ('Mz [-]', None, 'vehicle_torque_setpoint.xyz[2]', None),

    'motor0': ('m0 [-]', None, 'actuator_motors.control[0]', None),
    'motor1': ('m1 [-]', None, 'actuator_motors.control[1]', None),
    'motor2': ('m2 [-]', None, 'actuator_motors.control[2]', None),
    'motor3': ('m3 [-]', None, 'actuator_motors.control[3]', None),
}

# Ordered so a two-column, row-major layout pairs each position with its
# velocity:   x | vx     y | vy     z | vz     p | q     r | -
DEFAULT_SIGNALS = ('x', 'vx', 'y', 'vy', 'z', 'vz', 'p', 'q', 'r')

# Solid lines only — see guidelines/figures.md. Series are separated by weight
# and opacity: heaviest and faintest at the back, finest and most opaque in front.
SERIES_STYLE = [
    ('commanded', {'linewidth': 2.6, 'alpha': 0.55, 'zorder': 2}),
    ('estimated', {'linewidth': 1.6, 'alpha': 1.00, 'zorder': 3}),
    ('truth',     {'linewidth': 5.0, 'alpha': 0.25, 'zorder': 1}),
]


def fetch(ulog, t0, spec):
    """Return (t_seconds, values) for a "topic.field" spec, or None if absent."""
    if spec is None:
        return None
    topic, _, field = spec.partition('.')
    try:
        dataset = ulog.get_dataset(topic)
    except (KeyError, IndexError):
        return None
    if field not in dataset.data:
        return None
    # timestamps are uint64; some topics (groundtruth) start a few ms before
    # ulog.start_timestamp, and unsigned subtraction would wrap to ~1.8e19.
    t = (dataset.data['timestamp'].astype('float64') - float(t0)) / 1e6
    return t, dataset.data[field]


def best_series(ulog, t0, name):
    """Truth if the log has it, else the estimate. Returns (t, values) or None."""
    _ylabel, _cmd, est, truth = SIGNALS[name]
    for spec in (truth, est):
        series = fetch(ulog, t0, spec)
        if series is not None:
            return series
    return None


def _finish_axis(ax, ylabel, bottom, xlabel='time since log start [s]'):
    """Apply the house style to one axes."""
    ax.set_ylabel(ylabel, rotation=0, ha='right', va='center')
    ax.grid(False)
    niceplots.adjust_spines(ax, spines=['left', 'bottom'] if bottom else ['left'])
    if bottom:
        ax.set_xlabel(xlabel)


def save(fig, path):
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    # bbox_inches='tight' so offset spines and horizontal y-labels are not clipped
    fig.savefig(path, format='pdf', bbox_inches='tight')
    plt.close(fig)
    print(f'wrote {path}')


# --------------------------------------------------------------------- path

def plot_path(ulog, path=None):
    """Ground track coloured by time against the commanded path, with z below."""
    path = path or os.path.join(FIG_DIR, 'path.pdf')
    t0 = ulog.start_timestamp

    north, east, down = (best_series(ulog, t0, k) for k in ('x', 'y', 'z'))
    if north is None or east is None:
        print('note: no position data, skipping path plot', file=sys.stderr)
        return

    # commanded (trajectory_setpoint) -- the first spec of each SIGNALS entry
    cmd_n, cmd_e, cmd_d = (fetch(ulog, t0, SIGNALS[k][1]) for k in ('x', 'y', 'z'))

    t, n = north
    _, e = east

    fig, (ax, axz) = plt.subplots(
        2, 1, figsize=(9, 12), layout='constrained',
        gridspec_kw={'height_ratios': [2.2, 1]})

    # Commanded path first: wide and pale, so the flown track draws over it.
    if cmd_n is not None and cmd_e is not None:
        ax.plot(cmd_e[1], cmd_n[1], linewidth=6.0, alpha=0.30,
                color='#7f7f7f', zorder=1, label='commanded')

    # East on the horizontal, North on the vertical -- map convention.
    pts = np.array([e, n]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap='viridis',
                        norm=plt.Normalize(t.min(), t.max()),
                        linewidth=2.0, zorder=3)
    lc.set_array(t[:-1])
    ax.add_collection(lc)

    # TrajectorySetpoint fields are NaN when not commanded, and np.ptp is not
    # NaN-aware -- strip them before working out the limits.
    def finite(a):
        a = np.asarray(a, dtype=float)
        return a[np.isfinite(a)]

    allx = np.concatenate([finite(e), finite(cmd_e[1])]) if cmd_e is not None else finite(e)
    ally = np.concatenate([finite(n), finite(cmd_n[1])]) if cmd_n is not None else finite(n)
    pad = 0.05 * max(np.ptp(allx), np.ptp(ally), 1.0)
    ax.set_xlim(allx.min() - pad, allx.max() + pad)
    ax.set_ylim(ally.min() - pad, ally.max() + pad)
    ax.set_aspect('equal')
    ax.plot(e[0], n[0], 'o', color='k', markersize=7, zorder=5, label='start')
    ax.legend(loc='lower left', bbox_to_anchor=(0.0, 1.0), ncol=2, frameon=False)
    _finish_axis(ax, 'north\n[m]', bottom=True, xlabel='east [m]')

    # colour bar under the track -- see guidelines/figures.md
    cb = fig.colorbar(lc, ax=ax, orientation='horizontal', location='bottom',
                      shrink=0.7, pad=0.12)
    cb.set_label('flown path, time [s]')
    cb.outline.set_visible(False)

    if cmd_d is not None:
        axz.plot(cmd_d[0], cmd_d[1], linewidth=6.0, alpha=0.30,
                 color='#7f7f7f', zorder=1, label='commanded')
    if down is not None:
        axz.plot(down[0], down[1], linewidth=2.0, zorder=3, label='flown')
    axz.invert_yaxis()              # NED: down is positive, so up is up
    axz.legend(loc='lower left', bbox_to_anchor=(0.0, 1.0), ncol=2, frameon=False)
    _finish_axis(axz, 'z [m]', bottom=True)

    save(fig, path)


# ------------------------------------------------------------------- states

def plot_states(ulog, names=DEFAULT_SIGNALS, path=None, ncols=2):
    """Signals in `ncols` columns, filled row-major."""
    path = path or os.path.join(FIG_DIR, 'states.pdf')
    t0 = ulog.start_timestamp

    nrows = -(-len(names) // ncols)          # ceil
    fig, axes = plt.subplots(nrows, ncols, figsize=(6.5 * ncols, 2.9 * nrows),
                             layout='constrained', squeeze=False)

    for k, ax in enumerate(axes.ravel()):
        if k >= len(names):
            ax.set_visible(False)            # spare cell in the last row
            continue

        name = names[k]
        ylabel, *specs = SIGNALS[name]
        plotted = 0
        for spec, (label, style) in zip(specs, SERIES_STYLE):
            series = fetch(ulog, t0, spec)
            if series is None:
                continue
            ax.plot(series[0], series[1], label=label, **style)
            plotted += 1

        if plotted:
            ax.legend(loc='lower left', bbox_to_anchor=(0.0, 1.0),
                      ncol=3, frameon=False, fontsize='small')
        else:
            ax.text(0.5, 0.5, f'no data for "{name}"', ha='center',
                    transform=ax.transAxes)

        # bottom row of each column carries the x-axis
        bottom = (k + ncols) >= len(names)
        _finish_axis(ax, ylabel, bottom=bottom)

    save(fig, path)


# ---------------------------------------------------------------------- cli

def list_signals(ulog, log_path):
    present = {d.name for d in ulog.data_list}
    dur = (ulog.last_timestamp - ulog.start_timestamp) / 1e6
    print(f'{log_path}  ({dur:.1f} s)\n')
    for name, (ylabel, *specs) in SIGNALS.items():
        have = [s.partition('.')[0] in present for s in specs if s]
        mark = '+' if all(have) else ('~' if any(have) else '-')
        print(f'  {mark} {name:9} {ylabel}')
    print('\n  + all series present    ~ partial    - none logged')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('signals', nargs='*',
                        help='signal names for states.pdf (default: position, velocity, rates)')
    parser.add_argument('--log', default=DEFAULT_LOG,
                        help=f'.ulg path (default: {DEFAULT_LOG})')
    parser.add_argument('--fig-dir', default=FIG_DIR,
                        help=f'where to write the PDFs (default: {FIG_DIR})')
    parser.add_argument('--list', action='store_true',
                        help='list the signals available in this log and exit')
    args = parser.parse_args()

    if not os.path.exists(args.log):
        parser.error(f'no such log: {args.log}')
    ulog = ULog(args.log)

    if args.list:
        list_signals(ulog, args.log)
        return

    names = tuple(args.signals) if args.signals else DEFAULT_SIGNALS
    unknown = [s for s in names if s not in SIGNALS]
    if unknown:
        parser.error(f'unknown signal(s): {", ".join(unknown)}')

    plot_path(ulog, os.path.join(args.fig_dir, 'path.pdf'))
    plot_states(ulog, names, os.path.join(args.fig_dir, 'states.pdf'))


if __name__ == '__main__':
    main()
