import rclpy
import numpy as np
import transforms3d as tf
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Vector3Stamped, TransformStamped, PoseWithCovarianceStamped


class DynamicCameraPoseBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_camera_frame_tf2_Broadcaster')

        self._publisher = self.create_publisher(PoseWithCovarianceStamped,
                                                'camera_posecov',
                                                10)
        
        self.__pwcs = PoseWithCovarianceStamped()
        
        # Initialize timer for publishing transform
        self.__timer = self.create_timer(0.1, self.publish_camera_tf2)
        
        # Declare the transforms broadcaster
        self.__tf_broadcaster = TransformBroadcaster(self)
        
        # Declare the transform to broadcast
        self.__trans = TransformStamped()
        
        # Parent frame
        self.__trans.header.frame_id = 'gimbal'
        
        # Child frame
        self.__trans.child_frame_id = 'camera'   
        
        self.__trans.transform.translation.x = self.__pwcs.pose.pose.position.x = 0.0
        self.__trans.transform.translation.y = self.__pwcs.pose.pose.position.y = 0.0
        self.__trans.transform.translation.z = self.__pwcs.pose.pose.position.z = 0.0
        
        self.__initial_pose()
    
    def publish_camera_tf2(self):
        self.__trans.header.stamp = self.__pwcs.header.stamp = self.get_clock().now().to_msg()
        self.__tf_broadcaster.sendTransform(self.__trans)
        
        self.__pwcs.header.frame_id = "camera"
        self._publisher.publish(self.__pwcs)
        
        
    def __initial_pose(self):   
        self.__trans.transform.rotation.w = self.__pwcs.pose.pose.orientation.w =  0.5
        self.__trans.transform.rotation.x = self.__pwcs.pose.pose.orientation.x = -0.5
        self.__trans.transform.rotation.y = self.__pwcs.pose.pose.orientation.y =  0.5
        self.__trans.transform.rotation.z = self.__pwcs.pose.pose.orientation.z = -0.5
            
        self.__pwcs.pose.covariance = np.zeros(36)
    

def main():
    rclpy.init()
    node = DynamicCameraPoseBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()