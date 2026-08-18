"""Guidance: where should the vehicle be right now?

Frame convention throughout: NED. Down is positive, so 5 m above the ground
is z = -5.0.
"""

import math
from typing import NamedTuple, Optional, Sequence

Vec3 = tuple[float, float, float]


class Setpoint(NamedTuple):
    """
    What guidance wants at this instant.
    """
    position: Vec3
    yaw: float

def hold(x, y, z):
    """Constant position."""
    return lambda t: (x, y, z)

def const(value):
    """Constant scalar, for yaw."""
    return lambda t: value

def steps(values, dwell):
    """Piecewise-constant: values[i] for `dwell` seconds each, then hold the last."""
    def fn(t):
        return values[min(int(t // dwell), len(values) - 1)]
    return fn

def position_steps(base, axis, offsets, dwell):
    """Step one axis of `base` through `offsets`.  axis: 0=N, 1=E, 2=D."""
    def fn(t):
        p = list(base)
        p[axis] += offsets[min(int(t // dwell), len(offsets) - 1)]
        return tuple(p)
    return fn

def figure_eight(amplitude, altitude, period, lead_in=5.0):
    w = 2*math.pi/period
    def fn(t):
        s = max(0.0, t - lead_in)
        return (amplitude*math.sin(w*s),
                amplitude*math.sin(w*s)*math.cos(w*s),
                -altitude)
    fn.phase = lambda t: max(0.0, t - lead_in)     # expose the parameter
    return fn


def tangent_yaw(position_fn, dt=1e-3, t_start=0.0):
    """Yaw pointing along the path tangent.
    """
    def fn(t):
        s = max(t, t_start)
        x0, y0, _ = position_fn(s)
        x1, y1, _ = position_fn(s + dt)
        if x1 == x0 and y1 == y0:
            return 0.0
        return math.atan2(y1 - y0, x1 - x0)
    return fn



def square_waypoints(side: float = 5.0, altitude: float = 5.0) -> list[Vec3]:
    """
    Build the square, as a list of NED waypoints.
    """
    waypoints = [(0.,0.,-altitude), 
                 (side, 0., -altitude),
                 (side, side, -altitude),
                 (0., side, -altitude),
                 (0., 0., -altitude)]
    
    return waypoints

class ManeuverGuidance:
    """
    Performs maneuver from given position fn
    """

    def __init__(self, position_fn, yaw_fn, duration):
        """
        position_fn and yaw_fn take arg time and return position and yaw tuples
        """
        self._position_fn = position_fn
        self._yaw_fn = yaw_fn
        self._duration = duration
        self._t = 0.0

    def update(self, t, filler=None) -> Setpoint:
        self._t = float(t)
        pos = self._position_fn(t)
        return Setpoint(tuple(float(v) for v in pos), float(self._yaw_fn(self._t)))

    @property
    def target(self) -> Vec3:
        pos = self._position_fn(self._t)
        return tuple(float(v) for v in pos)

    @property
    def finished(self) -> bool:
        return self._duration is not None and self._t >= self._duration

    def reset(self) -> None:
        self._t = 0.0

    def distance_to_target(self, position) -> float:
        """Tracking error, for logging."""
        return math.dist(position, self.target)
        


class WaypointGuidance:
    """
    Walks a list of waypoints, advancing when the vehicle arrives.
    Arrival is judged on measured position.
    """

    def __init__(self, waypoints: Sequence[Vec3],
                 tolerance: float = 0.3, yaw: float = 0.0):
        if not waypoints:
            raise ValueError('waypoints must not be empty')

        self._waypoints = list(waypoints)
        self._tolerance = float(tolerance)
        self._yaw = float(yaw)
        self._index = 0

    @property
    def finished(self) -> bool:
        """
        True once the last waypoint has been reached.
        The node reads this to decide when to stop and land.
        """
        return self.index >= len(self._waypoints)

    @property
    def index(self) -> int:
        """Which leg we are on. Useful for logging and for plots later."""
        return self._index

    @property
    def target(self) -> Vec3:
        """The waypoint currently being flown to.

        Clamps to the last waypoint once finished, so this is always safe to
        read and always returns somewhere sensible to sit.
        """
        if self.finished:
            return self._waypoints[-1]
        return self._waypoints[self.index]

    def distance_to_target(self, position: Vec3) -> float:
        """Euclidean distance from `position` to the current target, in metres.

        """
        return math.dist(position, self.target)

    def update(self, t: float, position: Optional[Vec3]) -> Setpoint:
        """Advance if we have arrived, then return the setpoint to command.

        Args:
            t:        seconds since the guidance started. Unused by the plain
                      waypoint sequencer, but part of the signature from day
                      one so a time-parameterised trajectory (Phase 5) drops in
                      without changing the caller.
            position: measured NED position, or None if no estimate has
                      arrived yet.

        Returns:
            The Setpoint to publish this tick. NEVER returns None -- a gap in
            the setpoint stream longer than COM_OF_LOSS_T (1.0 s) makes PX4
            declare offboard lost. Even when finished, keep returning the last
            waypoint so the stream stays alive and the vehicle holds station.

        """
        if position is None:
            return Setpoint(self.target, self._yaw)
        
        if not self.finished and self.distance_to_target(position) < self._tolerance: # advance index
            self._index += 1

        return Setpoint(self.target, self._yaw)
            


    def reset(self) -> None:
        """Return to the first waypoint.

        Lets you re-fly the pattern without restarting the node, and keeps unit
        tests independent of each other.
        """

        self._index = 0

def _figure_eight():
    lead = 5.0
    pf = figure_eight(5.0, 5.0, 20.0, lead_in=lead)
    return ManeuverGuidance(pf, tangent_yaw(pf, t_start=lead), duration=50)



MANEUVERS = {
  'hover':     lambda: ManeuverGuidance(hold(0,0,-5), const(0.0), duration=60),
  'yaw_steps': lambda: ManeuverGuidance(hold(0,0,-5),
                          steps([0.0, math.pi/2, math.pi, 0.0], dwell=8), duration=40),
  'step_y':    lambda: ManeuverGuidance(position_steps((0,0,-5), 1, [0,2,-2,0], 8),
                          const(0.0), duration=40),
  'step_x':    lambda: ManeuverGuidance(position_steps((0,0,-5), 0, [0,2,-2,0], 8),
                          const(0.0), duration=40),
  'step_z':    lambda: ManeuverGuidance(position_steps((0,0,-5), 2, [0,-2,2,0], 8),
                          const(0.0), duration=40),
    'figure_eight': _figure_eight,
    'square': lambda: WaypointGuidance(square_waypoints())
}



        