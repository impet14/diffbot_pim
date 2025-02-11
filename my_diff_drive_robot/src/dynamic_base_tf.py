#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class DynamicBaseTfPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_base_tf_publisher')
        self.br = TransformBroadcaster(self)
        # Publish at 10 Hz for demonstration purposes
        self.timer = self.create_timer(0.1, self.publish_transform)

    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)
        self.get_logger().info('Published dynamic transform with stamp: %s' % t.header.stamp)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicBaseTfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
