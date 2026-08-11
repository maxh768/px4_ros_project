# node subscribing to the local position of vehicle

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition


px4_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

class LocalPositionEcho(Node):
    def __init__(self):
        super().__init__('local_position_echo')

        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1', # topic to subscribe to
            self.on_local_position,
            px4_qos # QoS profile
        )

    def on_local_position(self, msg):
        self.get_logger().info(
            f'NED  x={msg.x:+7.2f}  y={msg.y:+7.2f}  z={msg.z:+7.2f}   '
            f'xy_valid={msg.xy_valid} z_valid={msg.z_valid}',
            throttle_duration_sec=1.0,
        )

def main(args=None):
    rclpy.init(args=args)

    node = LocalPositionEcho()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()

