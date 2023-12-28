#! usr/env/python  /home/robotics20/franka_ros2_ws/.conda


import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class JointPublisher(Node):

    def __init__(self):
        super().__init__('joit_publisher')
        self.publisher_ = self.create_publisher(JointState, '/human/joint_states', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.name = "jRightSholder_rotx"
        msg.position = 1
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.name)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    joint_publisher = JointPublisher()

    rclpy.spin(joint_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()