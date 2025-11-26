# Copyright (c) 2025 AUV Team
# MIT License

"""Position/Direction node for AUV simulation.

This node subscribes to total_force, calculates AUV position, velocity,
acceleration, and direction, then publishes posi_data.
"""

import math
import rclpy
from rclpy.node import Node
from auv_interfaces.msg import PositionData, TotalForce


class PositionNode(Node):
    """Node that calculates AUV position and orientation from forces."""

    def __init__(self):
        """Initialize the position node."""
        super().__init__('position_node')

        # Declare parameters
        self.declare_parameter('mass', 30.0)          # AUV mass in kg
        self.declare_parameter('inertia_x', 2.0)      # Moment of inertia kg*m^2
        self.declare_parameter('inertia_y', 2.0)
        self.declare_parameter('inertia_z', 1.0)
        self.declare_parameter('update_rate', 100.0)  # Hz
        self.declare_parameter('buoyancy', 294.3)     # Buoyancy force in N

        # Get parameters
        self.mass = self.get_parameter(
            'mass').get_parameter_value().double_value
        self.inertia_x = self.get_parameter(
            'inertia_x').get_parameter_value().double_value
        self.inertia_y = self.get_parameter(
            'inertia_y').get_parameter_value().double_value
        self.inertia_z = self.get_parameter(
            'inertia_z').get_parameter_value().double_value
        self.buoyancy = self.get_parameter(
            'buoyancy').get_parameter_value().double_value
        update_rate = self.get_parameter(
            'update_rate').get_parameter_value().double_value

        # State variables
        self.position = [0.0, 0.0, 0.0]      # x, y, z in meters
        self.velocity = [0.0, 0.0, 0.0]      # vx, vy, vz in m/s
        self.acceleration = [0.0, 0.0, 0.0]  # ax, ay, az in m/s^2
        self.orientation = [0.0, 0.0, 0.0]   # roll, pitch, yaw in radians
        self.angular_velocity = [0.0, 0.0, 0.0]  # wx, wy, wz in rad/s

        # Total forces and torques (sum of all forces)
        self.total_force = [0.0, 0.0, 0.0]
        self.total_torque = [0.0, 0.0, 0.0]

        # Drag forces (from drag node)
        self.drag_force = [0.0, 0.0, 0.0]
        self.drag_torque = [0.0, 0.0, 0.0]

        # Motor forces (from motor node)
        self.motor_force = [0.0, 0.0, 0.0]
        self.motor_torque = [0.0, 0.0, 0.0]

        # Time step
        self.dt = 1.0 / update_rate

        # Subscribers
        self.total_force_sub = self.create_subscription(
            TotalForce,
            'totle_force',
            self.total_force_callback,
            10)

        self.drag_force_sub = self.create_subscription(
            TotalForce,
            'drag_force',
            self.drag_force_callback,
            10)

        # Publisher
        self.position_pub = self.create_publisher(
            PositionData,
            'posi_data',
            10)

        # Timer for physics update
        self.timer = self.create_timer(self.dt, self.update_physics)

        self.get_logger().info('Position node initialized')

    def total_force_callback(self, msg: TotalForce):
        """Store motor forces from total_force topic.

        Args:
            msg: Total force message from motor node
        """
        self.motor_force = [msg.force_x, msg.force_y, msg.force_z]
        self.motor_torque = [msg.torque_rx, msg.torque_ry, msg.torque_rz]

    def drag_force_callback(self, msg: TotalForce):
        """Store drag forces from drag node.

        Args:
            msg: Drag force message
        """
        self.drag_force = [msg.force_x, msg.force_y, msg.force_z]
        self.drag_torque = [msg.torque_rx, msg.torque_ry, msg.torque_rz]

    def update_physics(self):
        """Update physics simulation and publish position data."""
        # Calculate total forces (motor + drag + gravity + buoyancy)
        gravity = 9.81 * self.mass  # Weight in N (negative z direction)

        # Sum all forces
        self.total_force[0] = self.motor_force[0] + self.drag_force[0]
        self.total_force[1] = self.motor_force[1] + self.drag_force[1]
        # z: motor + drag + buoyancy - gravity
        self.total_force[2] = (
            self.motor_force[2] + self.drag_force[2] +
            self.buoyancy - gravity
        )

        # Sum all torques
        self.total_torque[0] = self.motor_torque[0] + self.drag_torque[0]
        self.total_torque[1] = self.motor_torque[1] + self.drag_torque[1]
        self.total_torque[2] = self.motor_torque[2] + self.drag_torque[2]

        # Calculate linear acceleration: F = ma -> a = F/m
        self.acceleration[0] = self.total_force[0] / self.mass
        self.acceleration[1] = self.total_force[1] / self.mass
        self.acceleration[2] = self.total_force[2] / self.mass

        # Calculate angular acceleration: T = I*alpha -> alpha = T/I
        angular_acceleration = [
            self.total_torque[0] / self.inertia_x,
            self.total_torque[1] / self.inertia_y,
            self.total_torque[2] / self.inertia_z,
        ]

        # Update velocity: v = v0 + a*dt
        self.velocity[0] += self.acceleration[0] * self.dt
        self.velocity[1] += self.acceleration[1] * self.dt
        self.velocity[2] += self.acceleration[2] * self.dt

        # Update angular velocity
        self.angular_velocity[0] += angular_acceleration[0] * self.dt
        self.angular_velocity[1] += angular_acceleration[1] * self.dt
        self.angular_velocity[2] += angular_acceleration[2] * self.dt

        # Update position: x = x0 + v*dt
        self.position[0] += self.velocity[0] * self.dt
        self.position[1] += self.velocity[1] * self.dt
        self.position[2] += self.velocity[2] * self.dt

        # Update orientation
        self.orientation[0] += self.angular_velocity[0] * self.dt
        self.orientation[1] += self.angular_velocity[1] * self.dt
        self.orientation[2] += self.angular_velocity[2] * self.dt

        # Normalize angles to [-pi, pi]
        for i in range(3):
            while self.orientation[i] > math.pi:
                self.orientation[i] -= 2 * math.pi
            while self.orientation[i] < -math.pi:
                self.orientation[i] += 2 * math.pi

        # Publish position data
        msg = PositionData()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.position_x = self.position[0]
        msg.position_y = self.position[1]
        msg.position_z = self.position[2]

        msg.velocity_x = self.velocity[0]
        msg.velocity_y = self.velocity[1]
        msg.velocity_z = self.velocity[2]

        msg.acceleration_x = self.acceleration[0]
        msg.acceleration_y = self.acceleration[1]
        msg.acceleration_z = self.acceleration[2]

        msg.roll = self.orientation[0]
        msg.pitch = self.orientation[1]
        msg.yaw = self.orientation[2]

        msg.angular_velocity_x = self.angular_velocity[0]
        msg.angular_velocity_y = self.angular_velocity[1]
        msg.angular_velocity_z = self.angular_velocity[2]

        self.position_pub.publish(msg)


def main(args=None):
    """Run the position node."""
    rclpy.init(args=args)
    node = PositionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
