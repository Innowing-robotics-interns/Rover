import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import time

def main(args=None):
    rclpy.init(args=args)
    node = Node('position_listener')
    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer, node)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # Process incoming messages
            try:
                # Lookup the latest transform
                transform: TransformStamped = tf_buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time())
                # Extract position and rotation
                position = transform.transform.translation
                rotation = transform.transform.rotation
                # Print position and rotation
                node.get_logger().info(f'Position: [{position.x}, {position.y}, {position.z}]')
                node.get_logger().info(f'Rotation: [{rotation.x}, {rotation.y}, {rotation.z}, {rotation.w}]')
            except Exception as e:
                node.get_logger().error(f'Could not transform base_footprint to map: {e}')
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()