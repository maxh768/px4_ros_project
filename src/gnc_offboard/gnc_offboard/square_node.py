"""
fly a square using PX4's own position controller.

"""

from enum import Enum, auto

import rclpy
from rclpy.node import Node

from gnc_offboard.px4_interface import Px4Interface
from gnc_offboard.guidance import WaypointGuidance, square_waypoints


class State(Enum):
    """
    Where we are in the handshake.

    Every transition out of REQUEST_* is gated on *observed* vehicle_status,
    never on elapsed time alone. Sending a command is not evidence it worked.
    """
    STREAMING = auto()          # publishing setpoints, nothing requested yet
    REQUEST_OFFBOARD = auto()   # asking for offboard, waiting for nav_state
    REQUEST_ARM = auto()        # asking to arm, waiting for arming_state
    FLYING = auto()             # walking the waypoints
    DONE = auto()               # finished; hold station over the last waypoint
    FAULT = auto()              # PX4 dropped us; stop commanding, report


class SquareNode(Node):

    def __init__(self):
        super().__init__('square_node')

        # Parameters
        self.declare_parameter('side', 5.0)             # square edge, metres
        self.declare_parameter('altitude', 5.0)         # height, positive up
        self.declare_parameter('tolerance', 0.3)        # arrival radius, metres
        self.declare_parameter('rate_hz', 50.0)         # setpoint stream rate
        self.declare_parameter('warmup_s', 1.0)         # stream before requesting
        self.declare_parameter('command_period_s', 0.5) # retry interval

        self._side = self.get_parameter('side').value
        self._altitude = self.get_parameter('altitude').value
        self._tolerance = self.get_parameter('tolerance').value
        self._rate_hz = self.get_parameter('rate_hz').value
        self._warmup_s = self.get_parameter('warmup_s').value
        self._command_period_s = self.get_parameter('command_period_s').value

        self.interface = Px4Interface(self)  # interface to talk to Px4
        # guidance code
        self._guidance = WaypointGuidance(
            square_waypoints(self._side, self._altitude),
            tolerance=self._tolerance)

        self._state = State.STREAMING
        self._start_ns = self.get_clock().now().nanoseconds
        self._last_command_ns = None

        # global timer
        self._timer = self.create_timer(1.0 / self._rate_hz, self._on_timer)

        self.get_logger().info(
            f'square: {self._side} m at {self._altitude} m, '
            f'tol {self._tolerance} m, {self._rate_hz} Hz')

    # ------------------------------------------------------------------ utils

    def _elapsed(self) -> float:
        """Seconds since the node started.

        Used for the warm-up delay and passed to guidance as `t`.
        """
        return (self.get_clock().now().nanoseconds - self._start_ns) / 1e9

    def _set_state(self, new: State) -> None:
        """
        Log transition of state
        """
        if new is not self._state:
            self.get_logger().info(f'{self._state.name} -> {new.name}')
            self._state = new
            self._last_command_ns = None   # let the new state command at once

    def _due_for_command(self) -> bool:
        now = self.get_clock().now().nanoseconds
        if (self._last_command_ns is None
                or (now - self._last_command_ns) / 1e9 >= self._command_period_s):
            self._last_command_ns = now
            return True
        return False


    # --------------------------------------------------------------- the loop

    def _on_timer(self) -> None:
        self.interface.publish_offboard_mode('position')   # ALWAYS, every tick

        position = self.interface.position if self._state is State.FLYING else None
        pos, yaw = self._guidance.update(self._elapsed(), position)

        self.interface.publish_trajectory_setpoint(position=pos, yaw=yaw)
        self._step()



    def _step(self) -> None:
        """Advance the state machine by one tick."""

        # Lost contact with PX4. status_age is inf until the first message
        # arrives, so this stays quiet during STREAMING startup.
        if self._state is not State.STREAMING and self.interface.status_age > 1.0:
            self.get_logger().warning(
                f'no vehicle_status for {self.interface.status_age:.1f} s',
                throttle_duration_sec=1.0)

        if self._state is State.STREAMING:
            # PX4 wants a live setpoint stream before it will accept the mode,
            # and we need to have heard from it at least once.
            if self._elapsed() > self._warmup_s and self.interface.status_age < 1.0:
                self._set_state(State.REQUEST_OFFBOARD)

        elif self._state is State.REQUEST_OFFBOARD:
            if self.interface.offboard:
                self._set_state(State.REQUEST_ARM)
            elif self._due_for_command():
                self.get_logger().info('requesting offboard')
                self.interface.set_offboard_mode()

        elif self._state is State.REQUEST_ARM:
            if self.interface.armed:
                self._set_state(State.FLYING)
            elif self._due_for_command():
                self.get_logger().info('requesting arm')
                self.interface.arm()

        elif self._state is State.FLYING:
            # Failure conditions first: never keep walking waypoints at a
            # vehicle PX4 has already taken back.
            if self.interface.status_age > 1.0:
                self.get_logger().error('lost vehicle_status while flying')
                self._set_state(State.FAULT)

            elif not self.interface.offboard:
                nav = self.interface.status.nav_state if self.interface.status else '?'
                self.get_logger().error(f'offboard lost, nav_state={nav}')
                self._set_state(State.FAULT)

            elif self._guidance.finished:
                self._set_state(State.DONE)

            else:
                position = self.interface.position
                if position is not None:
                    self.get_logger().info(
                        f'leg {self._guidance.index}  '
                        f'{self._guidance.distance_to_target(position):.2f} m to go',
                        throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = SquareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()