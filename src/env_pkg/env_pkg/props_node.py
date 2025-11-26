# Copyright (c) 2025 AUV Team
# MIT License

"""Props node for AUV simulation.

This node simulates props/objects in the environment and publishes
their positions for the camera simulation node.
"""

import rclpy
from rclpy.node import Node
from auv_interfaces.msg import ObjectsData


class PropsNode(Node):
    """Node that simulates props/objects in the environment."""

    def __init__(self):
        """Initialize the props node."""
        super().__init__('props_node')

        # Declare parameters for objects
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # Initialize objects list (can be configured via parameters or service)
        self.objects = [
            {'id': 1, 'type': 'gate', 'x': 5.0, 'y': 0.0, 'z': -2.0},
            {'id': 2, 'type': 'buoy', 'x': 10.0, 'y': 2.0, 'z': -1.0},
            {'id': 3, 'type': 'bin', 'x': 15.0, 'y': -1.0, 'z': -3.0},
            {'id': 4, 'type': 'ball', 'x': 8.0, 'y': 1.0, 'z': -2.5},
        ]

        # Publisher for objects data
        self.objects_pub = self.create_publisher(
            ObjectsData,
            'objects_data',
            10)

        # Timer for publishing at specified rate
        publish_rate = self.get_parameter(
            'publish_rate').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.get_logger().info(
            f'Props node initialized with {len(self.objects)} objects')

    def timer_callback(self):
        """Publish objects data periodically."""
        msg = ObjectsData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.num_objects = len(self.objects)

        msg.object_ids = [obj['id'] for obj in self.objects]
        msg.object_types = [obj['type'] for obj in self.objects]
        msg.positions_x = [obj['x'] for obj in self.objects]
        msg.positions_y = [obj['y'] for obj in self.objects]
        msg.positions_z = [obj['z'] for obj in self.objects]

        self.objects_pub.publish(msg)


def main(args=None):
    """Run the props node."""
    rclpy.init(args=args)
    node = PropsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
