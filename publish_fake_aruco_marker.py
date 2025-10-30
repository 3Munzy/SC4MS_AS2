#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import random

class FakeArucoPublisher(Node):
    def __init__(self):
        super().__init__('fake_aruco_marker_publisher')
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        
        # Publish every 5 seconds
        self.timer = self.create_timer(5.0, self.publish_marker)

        # Position bounds
        self.fixed_x = 0.8
        self.y_min, self.y_max = -0.7, 0.7
        self.z_min, self.z_max = 0.2, 0.9

        self.get_logger().info("Fake ArUco Publisher started: teleporting marker every 5 seconds.")

    def publish_marker(self):
        # Random Y and Z positions
        rand_y = random.uniform(self.y_min, self.y_max)
        rand_z = random.uniform(self.z_min, self.z_max)

        # Create the marker
        marker = Marker()
        marker.header.frame_id = 'world'  # match PBVS frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'fake_aruco'
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Position (teleports)
        marker.pose.position.x = self.fixed_x
        marker.pose.position.y = rand_y
        marker.pose.position.z = rand_z

        # Orientation (same as your static one)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Size
        marker.scale.x = 0.01
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Color
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0  # persists until next publish

        # Publish
        self.marker_pub.publish(marker)
        self.get_logger().info(
            f"Teleported fake ArUco marker to (x={marker.pose.position.x:.2f}, "
            f"y={marker.pose.position.y:.2f}, z={marker.pose.position.z:.2f})"
        )

def main():
    rclpy.init()
    node = FakeArucoPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
