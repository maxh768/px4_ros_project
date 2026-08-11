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
    """Build the Phase 1 square, as a list of NED waypoints.

    A free function rather than a method because the *geometry* is a separate
    concern from the *sequencing*. In later phases you swap this for a
    figure-eight or a sampled min-snap path, and WaypointGuidance is untouched.

    `altitude` is given as a positive height above the origin; the returned
    waypoints carry it as negative z, because NED.

    The path, starting and ending over the origin:

        climb   (0,    0,    -alt)
        leg 1   (side, 0,    -alt)    north
        leg 2   (side, side, -alt)    east
        leg 3   (0,    side, -alt)    south
        leg 4   (0,    0,    -alt)    west, back to start

    TODO: return that list.
    """
    raise NotImplementedError


class WaypointGuidance:
    """Walks a list of waypoints, advancing when the vehicle arrives.

    Arrival is judged on measured *position*, never on elapsed time. A timer
    would appear to work in Phase 1 and quietly break in Phase 2, when your own
    controller flies the same path at a different speed.
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
        """True once the last waypoint has been reached.

        The node reads this to decide when to stop and land. Note it does NOT
        mean "stop publishing" -- see the note in update().
        """
        return self._index >= len(self._waypoints)

    @property
    def index(self) -> int:
        """Which leg we are on. Useful for logging and for plots later."""
        return self._index

    @property
    def target(self) -> Vec3:
        """The waypoint currently being flown to.

        Clamps to the last waypoint once finished, so this is always safe to
        read and always returns somewhere sensible to sit.

        TODO: return the current waypoint, clamped to the final one.
        """
        raise NotImplementedError

    def distance_to_target(self, position: Vec3) -> float:
        """Euclidean distance from `position` to the current target, in metres.

        Split out from update() because it is the natural thing to assert on in
        a unit test, and the natural thing to log while tuning `tolerance`.

        TODO: implement. math.dist() does exactly this.
        """
        raise NotImplementedError

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

        Sketch:
            1. If `position` is None, we cannot judge arrival -- return the
               current target unchanged.
            2. If not finished and distance_to_target(position) < tolerance,
               advance the index.
            3. Return Setpoint(self.target, self._yaw).

        TODO: implement.
        """
        raise NotImplementedError

    def reset(self) -> None:
        """Return to the first waypoint.

        Lets you re-fly the pattern without restarting the node, and keeps unit
        tests independent of each other.

        TODO: implement.
        """
        raise NotImplementedError