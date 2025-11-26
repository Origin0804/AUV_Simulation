# Copyright (c) 2025 AUV Team
# MIT License

"""Motor node for AUV simulation.

This node subscribes to motors_data (PWM values), calculates thrust
from thrust curves, and publishes total force and torque.
"""

import math
import rclpy
from rclpy.node import Node
from auv_interfaces.msg import MotorsData, TotalForce


class MotorNode(Node):
    """Node that converts motor PWM to forces and torques."""

    def __init__(self):
        """Initialize the motor node."""
        super().__init__('motor_node')

        # Declare parameters
        self.declare_parameter('max_thrust', 50.0)    # Maximum thrust per motor (N)
        self.declare_parameter('motor_distance', 0.2)  # Distance from center (m)

        # Get parameters
        self.max_thrust = self.get_parameter(
            'max_thrust').get_parameter_value().double_value
        motor_distance = self.get_parameter(
            'motor_distance').get_parameter_value().double_value

        # Motor configuration (typical AUV 6-thruster config)
        # Motors 1-4: Horizontal thrusters (for X, Y motion and yaw)
        # Motors 5-6: Vertical thrusters (for Z motion and pitch/roll)
        # Motor positions relative to center, scaled by motor_distance
        d = motor_distance  # Scale factor for motor positions
        self.motor_config = [
            # Motor 1: Front-left horizontal, angled 45 degrees
            {'pos': [0.75 * d, 0.5 * d, 0.0], 'dir': [0.707, 0.707, 0.0]},
            # Motor 2: Front-right horizontal, angled 45 degrees
            {'pos': [0.75 * d, -0.5 * d, 0.0], 'dir': [0.707, -0.707, 0.0]},
            # Motor 3: Rear-left horizontal, angled 45 degrees
            {'pos': [-0.75 * d, 0.5 * d, 0.0], 'dir': [0.707, -0.707, 0.0]},
            # Motor 4: Rear-right horizontal, angled 45 degrees
            {'pos': [-0.75 * d, -0.5 * d, 0.0], 'dir': [0.707, 0.707, 0.0]},
            # Motor 5: Front vertical
            {'pos': [0.5 * d, 0.0, 0.0], 'dir': [0.0, 0.0, 1.0]},
            # Motor 6: Rear vertical
            {'pos': [-0.5 * d, 0.0, 0.0], 'dir': [0.0, 0.0, 1.0]},
        ]

        # Subscriber
        self.motors_sub = self.create_subscription(
            MotorsData,
            'motors_data',
            self.motors_callback,
            10)

        # Publisher
        self.force_pub = self.create_publisher(
            TotalForce,
            'totle_force',  # Note: keeping the original spelling from spec
            10)

        self.get_logger().info('Motor node initialized')

    def pwm_to_thrust(self, pwm: float) -> float:
        """Convert PWM value (-100 to 100) to thrust force.

        This uses a simplified thrust curve model.
        Real thrusters have non-linear characteristics.

        Args:
            pwm: PWM value from -100 to 100

        Returns:
            Thrust force in Newtons (positive or negative)
        """
        # Clamp PWM to valid range
        pwm = max(-100.0, min(100.0, pwm))

        # Simple quadratic thrust curve with linear scaling
        # Real thrusters: thrust ~ PWM^2, but signed
        normalized = pwm / 100.0
        # Using a simplified model: thrust = max_thrust * sign(pwm) * (pwm/100)^2
        thrust = self.max_thrust * math.copysign(normalized ** 2, normalized)

        return thrust

    def motors_callback(self, msg: MotorsData):
        """Convert motor PWM values to forces and torques.

        Args:
            msg: Motors data message with PWM values
        """
        pwm_values = [
            msg.motor1_pwm,
            msg.motor2_pwm,
            msg.motor3_pwm,
            msg.motor4_pwm,
            msg.motor5_pwm,
            msg.motor6_pwm,
        ]

        # Calculate thrust for each motor
        thrusts = [self.pwm_to_thrust(pwm) for pwm in pwm_values]

        # Calculate total force and torque
        total_force = [0.0, 0.0, 0.0]
        total_torque = [0.0, 0.0, 0.0]

        for i, (thrust, config) in enumerate(zip(thrusts, self.motor_config)):
            pos = config['pos']
            direction = config['dir']

            # Force contribution
            force = [thrust * d for d in direction]
            total_force[0] += force[0]
            total_force[1] += force[1]
            total_force[2] += force[2]

            # Torque contribution: T = r x F
            # Using cross product
            torque_x = pos[1] * force[2] - pos[2] * force[1]
            torque_y = pos[2] * force[0] - pos[0] * force[2]
            torque_z = pos[0] * force[1] - pos[1] * force[0]

            total_torque[0] += torque_x
            total_torque[1] += torque_y
            total_torque[2] += torque_z

        # Publish total force and torque
        force_msg = TotalForce()
        force_msg.header.stamp = self.get_clock().now().to_msg()
        force_msg.force_x = total_force[0]
        force_msg.force_y = total_force[1]
        force_msg.force_z = total_force[2]
        force_msg.torque_rx = total_torque[0]
        force_msg.torque_ry = total_torque[1]
        force_msg.torque_rz = total_torque[2]

        self.force_pub.publish(force_msg)


def main(args=None):
    """Run the motor node."""
    rclpy.init(args=args)
    node = MotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
