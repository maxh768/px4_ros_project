from pyulog import ULog
import numpy as np

SIGNALS = {

    # STATES
    'x':  'vehicle_local_position_groundtruth.x',
    'y':  'vehicle_local_position_groundtruth.y',
    'z':  'vehicle_local_position_groundtruth.z',
    'vx': 'vehicle_local_position_groundtruth.vx',
    'vy': 'vehicle_local_position_groundtruth.vy',
    'vz': 'vehicle_local_position_groundtruth.vz',

    'q0': 'vehicle_attitude_groundtruth.q[0]',
    'q1': 'vehicle_attitude_groundtruth.q[1]',
    'q2': 'vehicle_attitude_groundtruth.q[2]',
    'q3': 'vehicle_attitude_groundtruth.q[3]',

    'wx':  'vehicle_angular_velocity_groundtruth.xyz[0]',
    'wy':  'vehicle_angular_velocity_groundtruth.xyz[1]',
    'wz':  'vehicle_angular_velocity_groundtruth.xyz[2]',

    # INPUTS
    'm0': 'actuator_motors.control[0]',
    'm1': 'actuator_motors.control[1]',
    'm2': 'actuator_motors.control[2]',
    'm3': 'actuator_motors.control[3]',
}

QUAT = ('q0', 'q1', 'q2', 'q3')
STATES = ('x', 'y', 'z', 'vx', 'vy', 'vz', 'q0', 'q1', 'q2', 'q3', 'wx', 'wy', 'wz')
INPUTS = ('m0', 'm1', 'm2', 'm3')

K_T, OMEGA_MIN, OMEGA_MAX = 8.54858e-06, 150.0, 1000.0

NAV_OFFBOARD = 14      # VehicleStatus.NAVIGATION_STATE_OFFBOARD
ARMING_ARMED = 2       # VehicleStatus.ARMING_STATE_ARMED


def offboard_window(ulog, t0, margin=0.5):
    """(start, end) of the armed-and-offboard segment, trimmed by `margin`.

    Everything outside it is dead air for model validation: on the ground the
    model has no normal-force term, so it predicts falling while the vehicle
    sits still, and after offboard is lost PX4 flies its own failsafe.
    """
    s = ulog.get_dataset('vehicle_status')
    t = (s.data['timestamp'].astype('float64') - float(t0)) / 1e6
    ok = ((s.data['nav_state'] == NAV_OFFBOARD)
          & (s.data['arming_state'] == ARMING_ARMED))
    if not ok.any():
        raise ValueError('log contains no armed offboard segment')
    return t[ok][0] + margin, t[ok][-1] - margin


def control_to_thrust(c):
    """actuator_motors.control is normalised [0,1]; the gz bridge maps it
    linearly to rotor speed, and thrust goes as omega squared."""
    c = np.clip(np.nan_to_num(np.asarray(c, float)), 0.0, 1.0)
    return K_T * (OMEGA_MIN + c * (OMEGA_MAX - OMEGA_MIN)) ** 2


def quat_continuous(q):
    """q and -q are the same rotation. A sign flip between samples would make a
    linear interpolation swing through zero, so make the signs consistent."""
    q = np.asarray(q, float).copy()
    flip = np.cumprod(np.where(np.sum(q[1:] * q[:-1], axis=1) < 0, -1.0, 1.0))
    q[1:] *= flip[:, None]
    return q


def fetch(ulog, t0, spec):
    if spec is None:
        return None
    topic, _, field = spec.partition('.')
    try:
        dataset = ulog.get_dataset(topic)
    except (KeyError, IndexError):
        return None
    if field not in dataset.data:
        return None
    t = (dataset.data['timestamp'].astype('float64') - float(t0)) / 1e6
    return t, dataset.data[field]


def get_dataset(file, rate_hz=50.0):
    ulog = ULog(file)
    t0 = ulog.start_timestamp

    # 1. fetch
    series = {}
    for name in STATES:
        got = fetch(ulog, t0, SIGNALS[name])
        if got is None:
            raise KeyError(f'{name} missing from {file}')
        series[name] = got

    series_inputs = {}
    for name in INPUTS:
        got = fetch(ulog, t0, SIGNALS[name])
        if got is None:
            raise KeyError(f'{name} missing from {file}')
        series_inputs[name] = got

    tq = series['q0'][0]                       
    Q = quat_continuous(np.vstack([series[n][1] for n in QUAT]).T)
    for i, n in enumerate(QUAT):
        series[n] = (tq, Q[:, i])

    series_inputs = {n: (t, control_to_thrust(v))
                    for n, (t, v) in series_inputs.items()}


    # 2. a grid inside the span every signal covers, so we never extrapolate,
    #    and inside the armed-offboard segment, so there is no ground contact
    #    or failsafe flying in the data
    allser = {**series, **series_inputs}
    lo = max(t[0]  for t, _ in allser.values())
    hi = min(t[-1] for t, _ in allser.values())

    fly_lo, fly_hi = offboard_window(ulog, t0)
    lo, hi = max(lo, fly_lo), min(hi, fly_hi)

    t_grid = np.arange(lo, hi, 1.0 / rate_hz)

    # 3. interpolate each onto it, then stack column-wise
    cols_states = [np.interp(t_grid, series[n][0], series[n][1]) for n in STATES]
    cols_inputs = [np.interp(t_grid, series_inputs[n][0], series_inputs[n][1]) for n in INPUTS]

    X = np.vstack(cols_states).T
    U = np.vstack(cols_inputs).T

    qi = [STATES.index(n) for n in QUAT]
    X[:, qi] /= np.linalg.norm(X[:, qi], axis=1, keepdims=True)

    return t_grid, X, U
    

if __name__ == '__main__':
    import argparse, glob, os, re

    p = argparse.ArgumentParser()
    dir = 'flight_logs'
    out = 'data'
    rate_hz = 50

    os.makedirs(out, exist_ok=True)
    logs = sorted(f for f in glob.glob(os.path.join(dir, '*.ulg'))
                  if not os.path.islink(f))          # skip latest.ulg

    for f in logs:
        name = re.sub(r'^\d{4}-\d\d-\d\d_\d\d_\d\d_\d\d_', '',
                      os.path.splitext(os.path.basename(f))[0])
        
        t, X, U = get_dataset(f, rate_hz)

        np.savez_compressed(os.path.join(out, name + '.npz'),
                            t=t, X=X, U=U,
                            state_labels=np.array(STATES),
                            input_labels=np.array(INPUTS),
                            source=os.path.basename(f))
        print(f'{name:14} {t[-1]-t[0]:5.1f}s  X{X.shape}  U{U.shape}  '
              f'mean ΣT = {U.sum(1).mean():6.2f} N')
