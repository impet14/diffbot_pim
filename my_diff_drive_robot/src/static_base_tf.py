#!/usr/bin/env python3
"""
Static transform publisher for base_footprint → base_link.
This uses a latched publisher so its header stamp is set once.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class StaticBaseTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_base_transform_publisher')
        self.br = StaticTransformBroadcaster(self)
        self.publish_static_transform()

    def publish_static_transform(self):
        t = TransformStamped()
        # In ROS 2 static transforms, the timestamp does not need to update.
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'  # Parent frame (slam_toolbox base)
        t.child_frame_id = 'base_link'        # Your robot’s physical base
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)
        self.get_logger().info("Published static transform from base_footprint to base_link.")

def main(args=None):
    rclpy.init(args=args)
    node = StaticBaseTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
