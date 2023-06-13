import rclpy
import numpy as np
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from ..utils import px4_transforms as pxt
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped


class OdometryListener(Node):
    def __init__(self):
        super().__init__("drone_odometry")
        
        self.subscription = self.create_subscription(VehicleOdometry,
                                                     "/fmu/out/vehicle_odometry",
                                                     self.px4_to_ros_callback,
                                                     qos_profile_sensor_data)


        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 
                                               'drone_posecov',
                                               10)
        
        self.pwcs = PoseWithCovarianceStamped()
        

        
    
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
               
        self.pwcs.header.stamp = self.get_clock().now().to_msg()
        self.pwcs.header.frame_id = "drone"
        
        position_enu = pxt.transform_position(msg.position, "NED_2_ENU")
        self.pwcs.pose.pose.position.x = float(position_enu[0])
        self.pwcs.pose.pose.position.y = float(position_enu[1])
        self.pwcs.pose.pose.position.z = float(position_enu[2])
        
        q_enu_flu = pxt.px4_to_ros_orientation(msg.q)
        self.pwcs.pose.pose.orientation.w = float(q_enu_flu[0])
        self.pwcs.pose.pose.orientation.x = float(q_enu_flu[1]) 
        self.pwcs.pose.pose.orientation.y = float(q_enu_flu[2]) 
        self.pwcs.pose.pose.orientation.z = float(q_enu_flu[3])
        
        pose_variance_ned = np.hstack((msg.position_variance, msg.orientation_variance))
        pose_covariance_ned = np.eye(6) * pose_variance_ned
        pose_covariance_enu = pxt.transform_cov6d(pose_covariance_ned, "NED_2_ENU")
        self.pwcs.pose.covariance = pose_covariance_enu.flatten()
        
        self.publisher.publish(self.pwcs)
        
        
   
def main():
    rclpy.init()
    node = OdometryListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()