import rclpy
import numpy as np
import transforms3d as tf
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Vector3Stamped, TransformStamped, PoseWithCovarianceStamped


class DynamicCameraPoseBroadcaster(Node):

    def __init__(self, x, y, z):
        super().__init__('dynamic_camera_frame_tf2_Broadcaster')
        
        self._subscription = self.create_subscription(Vector3Stamped,
                                                     "/ZR30/get_gimbal_attitude",
                                                     self.camera_pose_callback,
                                                     10)


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
        self.__trans.header.frame_id = 'drone'
        
        # Child frame
        self.__trans.child_frame_id = 'camera'   
        
        self.__trans.transform.translation.x = self.__pwcs.pose.pose.position.x = float(x)
        self.__trans.transform.translation.y = self.__pwcs.pose.pose.position.y = float(y)
        self.__trans.transform.translation.z = self.__pwcs.pose.pose.position.z = float(z)
        
        
    
    def publish_camera_tf2(self):
        self.__trans.header.stamp = self.__pwcs.header.stamp = self.get_clock().now().to_msg()
        self.__tf_broadcaster.sendTransform(self.__trans)
        
        self.__pwcs.header.frame_id = "camera"
        self._publisher.publish(self.__pwcs)
        
        
    def camera_pose_callback(self, msg):
        """
        Transforms odometry provided from PX4 convention to ROS convention.
        (The FRD (NED) conventions are adopted on all PX4 topics unless explicitly 
        specified in the associated message definition. ROS follows the FLU (ENU) 
        convention) 

        Args:
            msg (Vector3Stamped): Camera orientation data. 
                                   
        Returns:
            None
            
        """
        self.get_logger().info('Camera orientation: {}'.format(msg.vector))
        q = tf.euler.euler2quat(msg.vector.x, msg.vector.y, msg.vector.z, 'rxyz')
        #self.get_logger().info('Camera q: {}'.format(q))
        self.__trans.transform.rotation.w = self.__pwcs.pose.pose.orientation.w = float(q[0])
        self.__trans.transform.rotation.x = self.__pwcs.pose.pose.orientation.x = float(q[1])
        self.__trans.transform.rotation.y = self.__pwcs.pose.pose.orientation.y = float(q[2])
        self.__trans.transform.rotation.z = self.__pwcs.pose.pose.orientation.z = float(q[3])
            
        #pose_variance_ned = np.hstack((msg.position_variance, msg.orientation_variance))
        #pose_covariance_ned = np.eye(6) * pose_variance_ned
        #pose_covariance_enu = pxt.transform_cov6d(pose_covariance_ned, "NED_2_ENU")
        #self.__pwcs.pose.covariance = pose_covariance_enu.flatten()


def main():
    rclpy.init()
    node = DynamicCameraPoseBroadcaster(x=0, y=0, z=1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()