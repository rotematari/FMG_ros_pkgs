import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped

class TFSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """
        order of links 
        - jLeftC7Shoulder_rotx
        - jLeftShoulder_rotx
        - jLeftShoulder_roty
        - jLeftShoulder_rotz
        - jLeftElbow_roty
        - jLeftElbow_rotz
        - jLeftWrist_rotx
        - jLeftWrist_rotz

        
        """
        try:
            # Replace 'child_frame_id' and 'base_frame_id' with the actual frame IDs
            trans = self.tf_buffer.lookup_transform('LeftShoulder', 'LeftForeArm_f1', rclpy.time.Time())
            self.get_logger().info('Transform: ' + str(trans))
        except Exception as e:
            self.get_logger().info('Could not transform: ' + str(e))

def main(args=None):
    rclpy.init(args=args)
    tf_subscriber = TFSubscriber()
    rclpy.spin(tf_subscriber)

    tf_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
