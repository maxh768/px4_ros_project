# communicates with px4 interface

import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleStatus, VehicleLocalPosition)


# QoS Profile to be used for px4 interface
px4_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class Px4Interface:

    def __init__(self, node: Node):
        self._node = node
        self._log = node.get_logger()

        self._pub_offboard = node.create_publisher( # publish the OffBoardControlMode
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)

        self._pub_setpoint = node.create_publisher( # give trajectory setpoint
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        self._pub_command = node.create_publisher( # publish vehicle commands
            VehicleCommand, '/fmu/in/vehicle_command',10)

        node.create_subscription( # get vehicle status
            VehicleStatus, '/fmu/out/vehicle_status_v4', self._on_status, px4_qos)

        node.create_subscription( # get pos
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._on_local_position, px4_qos
        )

        self._status = None
        self._status_ns = None
        self._position = None

        # control rungs
        self.RUNGS = ('position', 'velocity', 'acceleration', 'attitude', 'body_rate',
            'thrust_and_torque', 'direct_actuator')

    def _now_us(self) -> int:
        # PX4 timestamps
        return int(self._node.get_clock().now().nanoseconds / 1000)

    def publish_offboard_mode(self, rung:str) -> None:
        # pick the control rung and publish it to PX4 stack
        if rung not in self.RUNGS:
            raise ValueError(f'unknown rung {rung!r}; expected one of {self.RUNGS}')

        msg = OffboardControlMode()
        msg.timestamp = self._now_us()
        setattr(msg, rung, True)
        self._pub_offboard.publish(msg)


    def publish_trajectory_setpoint(self, position=None, velocity=None, acceleration=None,
                                    yaw=None) -> None:
        # publish the setpoint to px4 for control
        # unspecified gets NaN which means PX4 decides the setpoint
        nan3 = [math.nan] * 3

        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        # set the trajectory setpoints
        msg.position = [float(v) for v in position] if position is not None else nan3
        msg.velocity = [float(v) for v in velocity] if velocity is not None else nan3
        msg.acceleration = [float(v) for v in acceleration] if acceleration is not None else nan3
        msg.yaw = math.nan if yaw is None else float(yaw)
        # pub
        self._pub_setpoint.publish(msg)

    def send_command(self, command: int, **params) -> None:
        # send a vehicle command to PX4

        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = command
        for key, value in params.items():
            setattr(msg, key, float(value))

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._pub_command.publish(msg)

    # PX4_CUSTOM_MAIN_MODE_OFFBOARD, from px4_custom_mode.h — not exposed
    _MAIN_MODE_OFFBOARD = 6.0

    def set_offboard_mode(self) -> None:
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                          param1=1.0, param2=self._MAIN_MODE_OFFBOARD)

    def arm(self) -> None:
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self) -> None:
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    # inbound side:
    def _on_status(self, msg: VehicleStatus) -> None:
        self._status = msg
        self._status_ns = self._node.get_clock().now().nanoseconds

    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
        self._position = (msg.x, msg.y, msg.z)

    @property
    def status(self):
        return self._status

    @property
    def position(self):
        return self._position

    @property
    def armed(self) -> bool:
        return (self._status is not None
                and self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    @property
    def offboard(self) -> bool:
        return (self._status is not None
                and self._status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    @property
    def status_age(self) -> float:
        """Seconds since the last vehicle_status; inf if none has arrived."""
        if self._status_ns is None:
            return math.inf
        return (self._node.get_clock().now().nanoseconds - self._status_ns) / 1e9



