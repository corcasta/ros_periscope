import rclpy
import numpy as np
import transforms3d as tf
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped


class DynamicCameraFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_camera_frame_tf2_Broadcaster')
        
        # Declare the transforms broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)
        # Declare the transform to broadcast
        self._trans = TransformStamped()
        # Parent frame
        self._trans.header.frame_id = 'base_link'
        # Child frame
        self._trans.child_frame_id = 'camera'
        self.initial_pose()
        
        # Initialize timer for publishing transform
        self.timer = self.create_timer(0.1, self.publish_camera_tf2)
                
        # Subscribe to a drone_posecov topic and call broadcast_drone_pose
        # callback function on each message
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, 
                                                     'camera_posecov',
                                                     self.camera_pose_callback,
                                                     10)
        self.subscription  # prevent unused variable warning
    
    def initial_pose(self):
        """
            This just creates the starting point for the transforms
        """
        self._trans.header.stamp = self.get_clock().now().to_msg()
        
        self._trans.transform.translation.x = 0.0
        self._trans.transform.translation.y = 0.0
        self._trans.transform.translation.z = 1.0
        
        self._trans.transform.rotation.x = 0.0
        self._trans.transform.rotation.y = 0.0
        self._trans.transform.rotation.z = 0.0
        self._trans.transform.rotation.w = 1.0
        
        self._tf_broadcaster.sendTransform(self._trans)
        
    
    def publish_camera_tf2(self):
        self._trans.header.stamp = self.get_clock().now().to_msg()
        #print(self._trans)
        self._tf_broadcaster.sendTransform(self._trans)
        
        
    def camera_pose_callback(self, msg):
        """
        Updates drone pose state.        

        Args:
            msg (PoseWithCovarianceStamped): ros message of type PoseWithCovariance,
                                             containing drone pose info.
        
        Returns:
            None                                     
        """
        

        print(msg)        
        self._trans.transform.translation.x = msg.pose.pose.position.x
        self._trans.transform.translation.y = msg.pose.pose.position.y
        self._trans.transform.translation.z = msg.pose.pose.position.z
        
        self._trans.transform.rotation.x = msg.pose.pose.orientation.x
        self._trans.transform.rotation.y = msg.pose.pose.orientation.y
        self._trans.transform.rotation.z = msg.pose.pose.orientation.z
        self._trans.transform.rotation.w = msg.pose.pose.orientation.w
        
        #self.tf_broadcaster.sendTransform(t)
        
        #quaternion = [msg.pose.pose.position.x,
        #              msg.pose.pose.position.y, 
        #              msg.pose.pose.position.z, 
        #              msg.pose.pose.position.w]
        #
        #angles = tf.euler.quat2euler(quaternion, axes="sxyz")
        #
        #self.drone_phi   = angles[0]  
        #self.drone_theta = angles[1]
        #self.drone_psi   = angles[2]


def main():
    rclpy.init()
    node = DynamicCameraFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()