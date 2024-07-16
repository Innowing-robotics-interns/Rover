from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class FixedFrameBroadcaster(Node):

   def __init__(self):
       super().__init__('fixed_frame_tf2_broadcaster')
       self.tf_broadcaster = TransformBroadcaster(self)
       self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

   def broadcast_timer_callback(self):
       t = TransformStamped()
       t2 = TransformStamped()

       t.header.stamp = self.get_clock().now().to_msg()
       t.header.frame_id = 'base_footprint'
       t.child_frame_id = 'livox_frame'
       t.transform.translation.x = 0.075
       t.transform.translation.y = 0.0
       t.transform.translation.z = 0.0
       t.transform.rotation.x = 0.0
       t.transform.rotation.y = 0.0
       t.transform.rotation.z = 0.0
       t.transform.rotation.w = 1.0

       t2.header.stamp = self.get_clock().now().to_msg()
       t2.header.frame_id = 'base_footprint'
       t2.child_frame_id = 'base_link'
       t2.transform.translation.x = 0.0
       t2.transform.translation.y = 0.0
       t2.transform.translation.z = 0.0
       t2.transform.rotation.x = 0.0
       t2.transform.rotation.y = 0.0
       t2.transform.rotation.z = 0.0
       t2.transform.rotation.w = 1.0

       self.tf_broadcaster.sendTransform(t)
       self.tf_broadcaster.sendTransform(t2)


def main():
    print('Tf initialised successfuly')
    rclpy.init()
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()