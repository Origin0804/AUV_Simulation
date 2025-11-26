# Copyright (c) 2025 AUV Team
# MIT License

"""Camera node for AUV simulation.

This node simulates a camera by subscribing to position data and objects
data, then calculating which objects are visible and publishing CV model.
"""

import math
import rclpy
from rclpy.node import Node
from auv_interfaces.msg import (
    PositionData, ObjectsData, CVModel, DetectedObject
)


class CameraNode(Node):
    """Node that simulates camera detection."""

    def __init__(self):
        """Initialize the camera node."""
        super().__init__('camera_node')

        # Declare parameters
        self.declare_parameter('fov_horizontal', 1.22)  # 70 degrees in radians
        self.declare_parameter('fov_vertical', 0.87)    # 50 degrees in radians
        self.declare_parameter('max_range', 20.0)       # Maximum detection range
        self.declare_parameter('min_range', 0.3)        # Minimum detection range
        self.declare_parameter('publish_rate', 30.0)    # Hz

        # Get parameters
        self.fov_horizontal = self.get_parameter(
            'fov_horizontal').get_parameter_value().double_value
        self.fov_vertical = self.get_parameter(
            'fov_vertical').get_parameter_value().double_value
        self.max_range = self.get_parameter(
            'max_range').get_parameter_value().double_value
        self.min_range = self.get_parameter(
            'min_range').get_parameter_value().double_value
        publish_rate = self.get_parameter(
            'publish_rate').get_parameter_value().double_value

        # Current state
        self.position = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.objects = []

        # Subscribers
        self.position_sub = self.create_subscription(
            PositionData,
            'posi_data',
            self.position_callback,
            10)

        self.objects_sub = self.create_subscription(
            ObjectsData,
            'objects_data',
            self.objects_callback,
            10)

        # Publisher
        self.cv_pub = self.create_publisher(
            CVModel,
            'CV_modle',  # Note: keeping original spelling from spec
            10)

        # Timer for publishing
        self.timer = self.create_timer(
            1.0 / publish_rate, self.timer_callback)

        self.get_logger().info('Camera node initialized')

    def position_callback(self, msg: PositionData):
        """Update current position and orientation.

        Args:
            msg: Position data message
        """
        self.position = [msg.position_x, msg.position_y, msg.position_z]
        self.orientation = [msg.roll, msg.pitch, msg.yaw]

    def objects_callback(self, msg: ObjectsData):
        """Update objects list.

        Args:
            msg: Objects data message
        """
        self.objects = []
        for i in range(msg.num_objects):
            self.objects.append({
                'id': msg.object_ids[i],
                'type': msg.object_types[i],
                'x': msg.positions_x[i],
                'y': msg.positions_y[i],
                'z': msg.positions_z[i],
            })

    def transform_to_camera_frame(self, obj_world):
        """Transform object position from world frame to camera frame.

        Args:
            obj_world: Object position in world frame [x, y, z]

        Returns:
            Object position in camera frame [x, y, z] or None if behind camera
        """
        # Calculate relative position
        rel_x = obj_world[0] - self.position[0]
        rel_y = obj_world[1] - self.position[1]
        rel_z = obj_world[2] - self.position[2]

        # Apply rotation (simplified, using yaw only for this example)
        yaw = self.orientation[2]
        pitch = self.orientation[1]

        # Rotate around Z axis (yaw)
        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        x_yaw = rel_x * cos_yaw - rel_y * sin_yaw
        y_yaw = rel_x * sin_yaw + rel_y * cos_yaw

        # Rotate around Y axis (pitch)
        cos_pitch = math.cos(-pitch)
        sin_pitch = math.sin(-pitch)
        x_cam = x_yaw * cos_pitch + rel_z * sin_pitch
        z_cam = -x_yaw * sin_pitch + rel_z * cos_pitch
        y_cam = y_yaw

        return [x_cam, y_cam, z_cam]

    def is_in_fov(self, cam_pos):
        """Check if object is within camera field of view.

        Args:
            cam_pos: Object position in camera frame [x, y, z]

        Returns:
            Tuple of (is_visible, normalized_x, normalized_y, confidence)
        """
        x, y, z = cam_pos

        # Object must be in front of camera (positive x in camera frame)
        if x <= 0:
            return False, 0, 0, 0

        # Calculate distance
        distance = math.sqrt(x ** 2 + y ** 2 + z ** 2)
        if distance < self.min_range or distance > self.max_range:
            return False, 0, 0, 0

        # Calculate angles
        angle_horizontal = math.atan2(y, x)
        angle_vertical = math.atan2(z, x)

        # Check if within FOV
        if abs(angle_horizontal) > self.fov_horizontal / 2:
            return False, 0, 0, 0
        if abs(angle_vertical) > self.fov_vertical / 2:
            return False, 0, 0, 0

        # Calculate normalized image coordinates (0-1)
        norm_x = 0.5 + (angle_horizontal / self.fov_horizontal)
        norm_y = 0.5 - (angle_vertical / self.fov_vertical)

        # Calculate confidence based on distance
        confidence = 1.0 - (distance / self.max_range)
        confidence = max(0.1, min(1.0, confidence))

        return True, norm_x, norm_y, confidence

    def timer_callback(self):
        """Process objects and publish CV model."""
        cv_msg = CVModel()
        cv_msg.header.stamp = self.get_clock().now().to_msg()

        detected_objects = []

        for obj in self.objects:
            world_pos = [obj['x'], obj['y'], obj['z']]
            cam_pos = self.transform_to_camera_frame(world_pos)

            visible, norm_x, norm_y, confidence = self.is_in_fov(cam_pos)

            if visible:
                detected = DetectedObject()
                detected.object_id = obj['id']
                detected.object_type = obj['type']
                detected.x = norm_x
                detected.y = norm_y
                detected.width = 0.1  # Simplified fixed size
                detected.height = 0.1
                detected.confidence = confidence
                detected_objects.append(detected)

        cv_msg.target_detected = len(detected_objects) > 0
        cv_msg.num_objects = len(detected_objects)
        cv_msg.objects = detected_objects

        self.cv_pub.publish(cv_msg)


def main(args=None):
    """Run the camera node."""
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
