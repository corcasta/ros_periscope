import rclpy
import numpy as np
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from px4_msgs.msg import VehicleOdometry
from ..utils import px4_transforms as pxt
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped


class OdometryListener(Node):
    def __init__(self):
        super().__init__("drone_odometry")
        
        self._subscription = self.create_subscription(VehicleOdometry,
                                                     "/fmu/out/vehicle_odometry",
                                                     self.px4_to_ros_callback,
                                                     qos_profile_sensor_data)


        self._publisher = self.create_publisher(PoseWithCovarianceStamped,
                                                'drone_posecov',
                                                10)
        
        self.__pwcs = PoseWithCovarianceStamped()
        
        # Initialize timer for publishing transform
        self.__timer = self.create_timer(0.1, self.publish_drone_tf2)
        
        # Declare the transforms broadcaster
        self.__tf_broadcaster = TransformBroadcaster(self)
        
        # Declare the transform to broadcast
        self.__trans = TransformStamped()
        
        # Parent frame
        self.__trans.header.frame_id = 'odom'
        
        # Child frame
        self.__trans.child_frame_id = 'drone'    
        
        #self.__initial_pose()  
     
        
    def publish_drone_tf2(self):
        self.__trans.header.stamp = self.__pwcs.header.stamp = self.get_clock().now().to_msg()
        self.__tf_broadcaster.sendTransform(self.__trans)
        
        self.__pwcs.header.frame_id = "drone"
        self._publisher.publish(self.__pwcs)
    
    
    
    def __initial_pose(self):  
        """
        This method is intended to be used just for debugging purposes
        """ 
        self.__trans.transform.translation.x = self.__pwcs.pose.pose.position.x = 0.0
        self.__trans.transform.translation.y = self.__pwcs.pose.pose.position.y = 0.0
        self.__trans.transform.translation.z = self.__pwcs.pose.pose.position.z = 0.0
        
        self.__trans.transform.rotation.w = self.__pwcs.pose.pose.orientation.w = 1.0
        self.__trans.transform.rotation.x = self.__pwcs.pose.pose.orientation.x = 0.0
        self.__trans.transform.rotation.y = self.__pwcs.pose.pose.orientation.y = 0.0
        self.__trans.transform.rotation.z = self.__pwcs.pose.pose.orientation.z = 0.0
            
        self.__pwcs.pose.covariance = np.zeros(36)
        
        
    def px4_to_ros_callback(self, msg):
        """
        Transforms odometry provided from PX4 convention to ROS convention.
        (The FRD (NED) conventions are adopted on all PX4 topics unless explicitly 
        specified in the associated message definition. ROS follows the FLU (ENU) 
        convention) 

        Args:
            msg (VehicleOdometry): Vehicle odometry data. Fits ROS REP 147 for aerial vehicles.
                                   More INFO: https://docs.px4.io/main/en/msg_docs/VehicleOdometry.html
                                   
        Returns:
            None
            
        """
        
        position_enu = pxt.transform_position(msg.position, "NED_2_ENU")    
        self.__trans.transform.translation.x = self.__pwcs.pose.pose.position.x = float(position_enu[0])
        self.__trans.transform.translation.y = self.__pwcs.pose.pose.position.y = float(position_enu[1])
        self.__trans.transform.translation.z = self.__pwcs.pose.pose.position.z = float(position_enu[2])
        
        q_enu_flu = pxt.px4_to_ros_orientation(msg.q)
        self.__trans.transform.rotation.w = self.__pwcs.pose.pose.orientation.w = float(q_enu_flu[0])
        self.__trans.transform.rotation.x = self.__pwcs.pose.pose.orientation.x = float(q_enu_flu[1])
        self.__trans.transform.rotation.y = self.__pwcs.pose.pose.orientation.y = float(q_enu_flu[2])
        self.__trans.transform.rotation.z = self.__pwcs.pose.pose.orientation.z = float(q_enu_flu[3])
            
        pose_variance_ned = np.hstack((msg.position_variance, msg.orientation_variance))
        pose_covariance_ned = np.eye(6) * pose_variance_ned
        pose_covariance_enu = pxt.transform_cov6d(pose_covariance_ned, "NED_2_ENU")
        self.__pwcs.pose.covariance = pose_covariance_enu.flatten()
        
        
def main():
    rclpy.init()
    node = OdometryListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()