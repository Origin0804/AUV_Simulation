# Copyright (c) 2025 AUV Team
# MIT License

"""IMU (Inertial Measurement Unit) node for AUV simulation.

This node subscribes to position data and publishes IMU information
(acceleration and orientation) that users can subscribe to.
"""

import rclpy
from rclpy.node import Node
from auv_interfaces.msg import PositionData, IMUData


class IMUNode(Node):
    """Node that publishes IMU (acceleration and orientation) data."""

    def __init__(self):
        """Initialize the IMU node."""
        super().__init__('imu_node')

        # Declare parameters
        self.declare_parameter('publish_rate', 100.0)  # Hz

        publish_rate = self.get_parameter(
            'publish_rate').get_parameter_value().double_value

        # Current state
        self.acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0]  # roll, pitch, yaw

        # Subscriber
        self.position_sub = self.create_subscription(
            PositionData,
            'posi_data',
            self.position_callback,
            10)

        # Publisher
        self.imu_pub = self.create_publisher(
            IMUData,
            'imu_data',
            10)

        # Timer for publishing
        self.timer = self.create_timer(
            1.0 / publish_rate, self.timer_callback)

        self.get_logger().info('IMU node initialized')

    def position_callback(self, msg: PositionData):
        """Update state from position data.

        Args:
            msg: Position data message
        """
        self.acceleration = [
            msg.acceleration_x,
            msg.acceleration_y,
            msg.acceleration_z
        ]
        self.angular_velocity = [
            msg.angular_velocity_x,
            msg.angular_velocity_y,
            msg.angular_velocity_z
        ]
        self.orientation = [msg.roll, msg.pitch, msg.yaw]

    def timer_callback(self):
        """Publish IMU data."""
        imu_msg = IMUData()
        imu_msg.header.stamp = self.get_clock().now().to_msg()

        imu_msg.acceleration_x = self.acceleration[0]
        imu_msg.acceleration_y = self.acceleration[1]
        imu_msg.acceleration_z = self.acceleration[2]

        imu_msg.angular_velocity_x = self.angular_velocity[0]
        imu_msg.angular_velocity_y = self.angular_velocity[1]
        imu_msg.angular_velocity_z = self.angular_velocity[2]

        imu_msg.roll = self.orientation[0]
        imu_msg.pitch = self.orientation[1]
        imu_msg.yaw = self.orientation[2]

        self.imu_pub.publish(imu_msg)


def main(args=None):
    """Run the IMU node."""
    rclpy.init(args=args)
    node = IMUNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
