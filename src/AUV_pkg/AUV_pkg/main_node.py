# Copyright (c) 2025 AUV Team
# MIT License

"""Main node for AUV simulation - User programming interface.

This node is where users can program their AUV control logic.
It subscribes to sensor data (DVL, IMU, CV) and publishes motor commands.
"""

import rclpy
from rclpy.node import Node
from auv_interfaces.msg import DVLData, IMUData, CVModel, MotorsData


class MainNode(Node):
    """Main node for user programming - controls AUV based on sensor data."""

    def __init__(self):
        """Initialize the main node."""
        super().__init__('main_node')

        # Declare parameters
        self.declare_parameter('control_rate', 50.0)  # Hz

        control_rate = self.get_parameter(
            'control_rate').get_parameter_value().double_value

        # Sensor data storage
        self.dvl_data = None
        self.imu_data = None
        self.cv_data = None

        # Subscribers for sensor data
        self.dvl_sub = self.create_subscription(
            DVLData,
            'DVL_data',
            self.dvl_callback,
            10)

        self.imu_sub = self.create_subscription(
            IMUData,
            'imu_data',
            self.imu_callback,
            10)

        self.cv_sub = self.create_subscription(
            CVModel,
            'CV_modle',
            self.cv_callback,
            10)

        # Publisher for motor commands
        self.motors_pub = self.create_publisher(
            MotorsData,
            'motors_data',
            10)

        # Control loop timer
        self.timer = self.create_timer(
            1.0 / control_rate, self.control_loop)

        self.get_logger().info('Main node initialized - User programming node')

    def dvl_callback(self, msg: DVLData):
        """Store DVL data.

        Args:
            msg: DVL data message
        """
        self.dvl_data = msg

    def imu_callback(self, msg: IMUData):
        """Store IMU data.

        Args:
            msg: IMU data message
        """
        self.imu_data = msg

    def cv_callback(self, msg: CVModel):
        """Store CV model data.

        Args:
            msg: CV model message
        """
        self.cv_data = msg

    def control_loop(self):
        """Main control loop - users should modify this method.

        This is where users implement their AUV control logic.
        The method has access to:
        - self.dvl_data: Velocity information
        - self.imu_data: Acceleration and orientation
        - self.cv_data: Camera/vision detection results

        Users should calculate motor PWM values and publish them.
        """
        # Create motor command message
        motors_msg = MotorsData()
        motors_msg.header.stamp = self.get_clock().now().to_msg()

        # =====================================================
        # USER PROGRAMMING AREA - Modify the logic below
        # =====================================================

        # Example: Simple depth control
        # This is a basic example - replace with your control logic
        target_depth = -2.0  # Target depth in meters (negative is below surface)

        if self.imu_data is not None:
            # Simple proportional control for depth
            # In reality, you would use PID or more advanced control
            current_pitch = self.imu_data.pitch

            # Vertical motors control (motors 5 and 6)
            # Adjust based on pitch to maintain level
            base_thrust = 0.0  # Neutral buoyancy assumed
            pitch_correction = -current_pitch * 10.0

            motors_msg.motor5_pwm = base_thrust + pitch_correction
            motors_msg.motor6_pwm = base_thrust - pitch_correction

        if self.cv_data is not None and self.cv_data.target_detected:
            # Example: Simple target tracking
            # Adjust yaw to center target in view
            if len(self.cv_data.objects) > 0:
                target = self.cv_data.objects[0]
                # Target x is 0-1, 0.5 is center
                error_x = target.x - 0.5

                # Use horizontal motors for yaw adjustment
                yaw_thrust = error_x * 20.0
                motors_msg.motor1_pwm = yaw_thrust
                motors_msg.motor2_pwm = -yaw_thrust
                motors_msg.motor3_pwm = -yaw_thrust
                motors_msg.motor4_pwm = yaw_thrust

        # =====================================================
        # END USER PROGRAMMING AREA
        # =====================================================

        # Publish motor commands
        self.motors_pub.publish(motors_msg)


def main(args=None):
    """Run the main node."""
    rclpy.init(args=args)
    node = MainNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
