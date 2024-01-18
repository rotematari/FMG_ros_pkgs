import math
import numpy as np  
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations as tf
from scipy.spatial.transform import Rotation as R
from natnet.utils import calculate_vectors 
from natnet.NatnetReader import read_sample, init_natnetClient

class MoveArm(Node):
    """
    ROS2 Node for listening to frames and broadcasting joint states.
    """

    def __init__(self):
        super().__init__('model_human_arm_mover')


        # Create human JointState publisher
        self.joint_state_publisher = self.create_publisher(JointState, '/human/human/joint_states', 10)
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        """
        Timer callback to read and publish joint states.
        """
        try:
            # Read current location data
            locations = read_sample(natnet=self.natnet)
            CtoS, StoE, EtoW = calculate_vectors(locations)

            # Calculate shoulder rotation
            shoulder_rotation = self.calculate_joint_rotation(np.array([0, 1, 0]), StoE)
            shoulder_euler_angles = shoulder_rotation.as_euler('XYZ')

            # Calculate elbow rotation
            elbow_rotation = self.calculate_joint_rotation(StoE, EtoW)
            elbow_euler_angles = elbow_rotation.as_euler('XYZ')

            # Publish joint state message
            self.publish_joint_state(shoulder_euler_angles, elbow_euler_angles)

        except Exception as e:
            self.get_logger().info('Error in processing joint states: ' + str(e))

    def calculate_joint_rotation(self, reference_vector, target_vector):
        """
        Calculates the rotation required to align a reference vector to a target vector.
        """
        rotation_axis = np.cross(reference_vector, target_vector)
        rotation_angle = np.arccos(np.dot(reference_vector, target_vector))
        return R.from_rotvec(rotation_axis * rotation_angle)

    def publish_joint_state(self, shoulder_angles, elbow_angles):
        """
        Publishes a joint state message with given shoulder and elbow angles.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'jLeftShoulder_rotx',
            'jLeftShoulder_roty',
            'jLeftShoulder_rotz',
            'jLeftElbow_rotx',
            'jLeftElbow_roty',
            'jLeftElbow_rotz',
        ]
        msg.position = [
                        round(shoulder_angles[0],2),
                        round(shoulder_angles[1],2),
                        round(shoulder_angles[2],2),
                        round(elbow_angles[0],2),
                        round(elbow_angles[1],2),
                        round(elbow_angles[2],2),
                        ]
        self.joint_state_publisher.publish(msg)

def main():
    rclpy.init()
    node = MoveArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
