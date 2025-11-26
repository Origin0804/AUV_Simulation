# Copyright (c) 2025 AUV Team
# MIT License

"""Drag node for AUV simulation.

This node calculates fluid drag forces based on the AUV's velocity
using a simplified fluid dynamics model.
"""

import rclpy
from rclpy.node import Node
from auv_interfaces.msg import PositionData, TotalForce


class DragNode(Node):
    """Node that calculates drag forces based on velocity."""

    def __init__(self):
        """Initialize the drag node."""
        super().__init__('drag_node')

        # Declare parameters for drag calculation
        self.declare_parameter('drag_coefficient', 0.47)  # Sphere-like coefficient
        self.declare_parameter('fluid_density', 1025.0)   # Seawater density kg/m^3
        self.declare_parameter('reference_area', 0.25)    # Reference area m^2
        self.declare_parameter('angular_drag_coefficient', 0.1)  # Angular drag

        # Get parameters
        self.drag_coefficient = self.get_parameter(
            'drag_coefficient').get_parameter_value().double_value
        self.fluid_density = self.get_parameter(
            'fluid_density').get_parameter_value().double_value
        self.reference_area = self.get_parameter(
            'reference_area').get_parameter_value().double_value
        self.angular_drag_coefficient = self.get_parameter(
            'angular_drag_coefficient').get_parameter_value().double_value

        # Subscriber for position data (velocity)
        self.position_sub = self.create_subscription(
            PositionData,
            'posi_data',
            self.position_callback,
            10)

        # Publisher for drag forces
        self.drag_pub = self.create_publisher(
            TotalForce,
            'drag_force',
            10)

        self.get_logger().info('Drag node initialized')

    def position_callback(self, msg: PositionData):
        """Calculate and publish drag forces based on velocity.

        Args:
            msg: Position data message containing velocity information
        """
        # Calculate drag force using: F = 0.5 * rho * Cd * A * v^2
        # Direction is opposite to velocity
        drag_msg = TotalForce()
        drag_msg.header.stamp = self.get_clock().now().to_msg()

        # Linear drag forces
        vx, vy, vz = msg.velocity_x, msg.velocity_y, msg.velocity_z
        k = 0.5 * self.fluid_density * self.drag_coefficient * self.reference_area

        # Drag is proportional to v^2, direction opposite to velocity
        drag_msg.force_x = -k * abs(vx) * vx
        drag_msg.force_y = -k * abs(vy) * vy
        drag_msg.force_z = -k * abs(vz) * vz

        # Angular drag (simplified model)
        wx = msg.angular_velocity_x
        wy = msg.angular_velocity_y
        wz = msg.angular_velocity_z
        k_angular = self.angular_drag_coefficient

        drag_msg.torque_rx = -k_angular * abs(wx) * wx
        drag_msg.torque_ry = -k_angular * abs(wy) * wy
        drag_msg.torque_rz = -k_angular * abs(wz) * wz

        self.drag_pub.publish(drag_msg)


def main(args=None):
    """Run the drag node."""
    rclpy.init(args=args)
    node = DragNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
