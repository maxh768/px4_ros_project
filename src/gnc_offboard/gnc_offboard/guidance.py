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
        