#!/usr/bin/env python3
"""Plot signals from a PX4 .ulg flight log.

Every signal is one line in SIGNALS below: a short name mapped to up to three
series -- what was commanded, what the estimator believed, and what was
actually true in simulation. Adding a new signal should never require touching
the plotting code.

    tools/plot_ulog.py --list
    tools/plot_ulog.py z vz
    tools/plot_ulog.py p q r --log flight_logs/2026-08-11_02_03_07.ulg --save rates.png

Field paths are "topic.field"; array members use pyulog's flattened form,
e.g. vehicle_angular_velocity.xyz[0].
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import niceplots
from pyulog import ULog

# See guidelines/figures.md — niceplots style, offset spines, horizontal
# y-labels, no grid, boxless legend, PDF output.
plt.style.use(niceplots.get_style())

DEFAULT_LOG = 'flight_logs/latest.ulg'
DEFAULT_FIG = 'debug/ulogdata.pdf'

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

# Solid lines only — see guidelines/figures.md. The three series are separated
# by weight and opacity instead: heaviest and faintest at the back, finest and
# most opaque in front.
SERIES_STYLE = [
    ('commanded', {'linewidth': 2.0, 'alpha': 1.0,}),
    ('estimated', {'linewidth': 2.0, 'alpha': 1.0,}),
    ('truth',     {'linewidth': 2.0, 'alpha': 1.0,}),
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


def plot(ulog, names, save=None):
    t0 = ulog.start_timestamp
    # niceplots sets font.size=18, so panels need room; constrained layout
    # accounts for artists placed outside the axes (the legends).
    fig, axes = plt.subplots(len(names), 1, sharex=True,
                             figsize=(11, 3.6 * len(names)), layout='constrained')
    axes = [axes] if len(names) == 1 else list(axes)

    for ax, name in zip(axes, names):
        ylabel, *specs = SIGNALS[name]
        plotted = 0
        for spec, (label, style) in zip(specs, SERIES_STYLE):
            series = fetch(ulog, t0, spec)
            if series is None:
                if spec is not None:
                    print(f'  note: {name}/{label} unavailable ({spec})', file=sys.stderr)
                continue
            ax.plot(series[0], series[1], label=label, **style)
            print(f'  {name:9} {label:10} {spec}')
            plotted += 1

        ax.set_ylabel(ylabel, rotation=0, ha='right', va='center')
        ax.grid(False)
        if plotted:
            # above the axes, so it can never sit on top of the data
            ax.legend(loc='lower left', bbox_to_anchor=(0.0, 1.0), ncol=3,
                      frameon=False)
        else:
            ax.text(0.5, 0.5, f'no data for "{name}"', ha='center', transform=ax.transAxes)

        # only the bottom panel carries the shared x-axis
        spines = ['left', 'bottom'] if ax is axes[-1] else ['left']
        niceplots.adjust_spines(ax, spines=spines)

    axes[-1].set_xlabel('time since log start [s]')

    if save:
        if not save.lower().endswith('.pdf'):
            save = os.path.splitext(save)[0] + '.pdf'
            print(f'note: figures are saved as PDF, writing {save}', file=sys.stderr)
        os.makedirs(os.path.dirname(save) or '.', exist_ok=True)
        # bbox_inches='tight' so offset spines and horizontal y-labels are
        # not clipped at the page edge
        fig.savefig(save, format='pdf', bbox_inches='tight')
        print(f'wrote {save}')
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('signals', nargs='*', help='signal names to plot (see --list)')
    parser.add_argument('--log', default=DEFAULT_LOG, help=f'.ulg path (default: {DEFAULT_LOG})')
    parser.add_argument('--save', nargs='?', const=DEFAULT_FIG, default=DEFAULT_FIG,
                        help=f'write a PDF instead of opening a window (default: {DEFAULT_FIG})')
    parser.add_argument('--list', action='store_true', help='list signals available in this log')
    args = parser.parse_args()

    if not os.path.exists(args.log):
        parser.error(f'no such log: {args.log}')
    ulog = ULog(args.log)

    if args.list or not args.signals:
        present = {d.name for d in ulog.data_list}
        print(f'{args.log}  ({ulog.last_timestamp - ulog.start_timestamp:.0f} us)\n')
        for name, (ylabel, *specs) in SIGNALS.items():
            have = [s.partition('.')[0] in present for s in specs if s]
            mark = '+' if all(have) else ('~' if any(have) else '-')
            print(f'  {mark} {name:9} {ylabel}')
        print('\n  + all series present    ~ partial    - none logged')
        return

    unknown = [s for s in args.signals if s not in SIGNALS]
    if unknown:
        parser.error(f'unknown signal(s): {", ".join(unknown)}')

    plot(ulog, args.signals, args.save)


if __name__ == '__main__':
    main()
