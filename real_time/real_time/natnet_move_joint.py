#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
import pandas as pd 
from math import pi 
import numpy as np 

import natnet.utils

from natnet.utils import calculate_angles_with_axes , calculate_vectors, calculate_full_circle_angles,calculate_euler_angles
from natnet.NatnetReader import read_sample, init_natnetClient

class JointPublisher(Node):

    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(JointState, '/human/human/joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


        self.natnet = init_natnetClient()

        self.natnet.run()

        # self.locations: {
        #     'chest':[],
        #     'shoulder':[],
        #     'elbow':[],
        #     'wrist':[],
        #     }

        

        # self.StoE_to_EtoW_angle_df = 

    def timer_callback(self):

        self.locations = read_sample(natnet=self.natnet)

        CtoS, StoE, EtoW = calculate_vectors(self.locations)
        print("StoE")
        print(StoE)

        self.StoE_angles = calculate_euler_angles(StoE) 
        print(self.StoE_angles)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'jLeftShoulder_rotx',
            # 'jLeftShoulder_roty',
            # 'jLeftShoulder_rotz',
            # 'jLeftElbow_rotz'
            ]
        msg.position = [
                        self.StoE_angles['X'],
                        # self.StoE_angles['Z'],
                        # -self.StoE_angles['Y'] -pi/2,
                        # self.StoE_to_EtoW_angle_df['rad'][self.i]
                        ]
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