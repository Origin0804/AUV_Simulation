# Copyright (c) 2025 AUV Team
# MIT License

"""DVL (Doppler Velocity Log) node for AUV simulation.

This node subscribes to position data and publishes velocity information
that users can subscribe to.
"""

import rclpy
from rclpy.node import Node
from auv_interfaces.msg import PositionData, DVLData


class DVLNode(Node):
    """Node that publishes DVL (velocity) data."""

    def __init__(self):
        """Initialize the DVL node."""
        super().__init__('dvl_node')

        # Declare parameters
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('noise_std', 0.01)     # Velocity noise std dev

        publish_rate = self.get_parameter(
            'publish_rate').get_parameter_value().double_value

        # Current velocity (from position data)
        self.velocity = [0.0, 0.0, 0.0]
        self.data_valid = False

        # Subscriber
        self.position_sub = self.create_subscription(
            PositionData,
            'posi_data',
            self.position_callback,
            10)

        # Publisher
        self.dvl_pub = self.create_publisher(
            DVLData,
            'DVL_data',
            10)

        # Timer for publishing
        self.timer = self.create_timer(
            1.0 / publish_rate, self.timer_callback)

        self.get_logger().info('DVL node initialized')

    def position_callback(self, msg: PositionData):
        """Update velocity from position data.

        Args:
            msg: Position data message
        """
        self.velocity = [msg.velocity_x, msg.velocity_y, msg.velocity_z]
        self.data_valid = True

    def timer_callback(self):
        """Publish DVL data."""
        dvl_msg = DVLData()
        dvl_msg.header.stamp = self.get_clock().now().to_msg()
        dvl_msg.velocity_x = self.velocity[0]
        dvl_msg.velocity_y = self.velocity[1]
        dvl_msg.velocity_z = self.velocity[2]
        dvl_msg.valid = self.data_valid

        self.dvl_pub.publish(dvl_msg)


def main(args=None):
    """Run the DVL node."""
    rclpy.init(args=args)
    node = DVLNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
