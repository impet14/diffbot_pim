#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class BaseLinkLaserTfPublisher(Node):
    def __init__(self):
        super().__init__('base_link_laser_tf_publisher')
        self.br = TransformBroadcaster(self)
        # Publish at 100 Hz
        self.timer = self.create_timer(0.01, self.publish_transform)
        self.get_logger().info('Publishing transform from base_link to laser_frame at 100 Hz.')

    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.02  # Adjust based on your laser mounting
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = BaseLinkLaserTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
