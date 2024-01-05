#!/usr/bin/env python3  

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
import pandas as pd 
from math import pi 
import numpy as np 
class JointPublisher(Node):

    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(JointState, '/human/human/joint_states', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.StoE_angles_df = pd.read_csv(r'build/move_joint/data/StoE_angles.csv')
        self.StoE_to_EtoW_angle_df = pd.read_csv(r'build/move_joint/data/StoE_to_EtoW_angle.csv')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'jLeftShoulder_roty',
            'jLeftShoulder_rotx',
            'jLeftShoulder_rotz',
            'jLeftElbow_rotz']
        msg.position = [
                        self.StoE_angles_df['X'][self.i],
                        self.StoE_angles_df['Y'][self.i]-pi,
                        -self.StoE_angles_df['Z'][self.i]+pi,
                        self.StoE_to_EtoW_angle_df['rad'][self.i]]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing to :%s position' % msg  )

        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    joint_publisher = JointPublisher()

    rclpy.spin(joint_publisher)

    joint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()