import rclpy
import numpy as np
from ..utils import px4_transforms as mtf

from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data


class OdometryListener(Node):
    """
    DEMO on How to interface PX4 data
    """
    def __init__(self):
        super().__init__("px4_sensors")
        
        self.subscription = self.create_subscription(VehicleOdometry, #SensorCombined,
                                                     "/fmu/out/vehicle_odometry",#"/fmu/out/sensor_combined",
                                                     self.demo_callback,
                                                     qos_profile_sensor_data)


        self.publisher = self.create_publisher(PoseWithCovarianceStamped, 
                                               'drone_posecov',
                                               10)
        
        self.pwcs = PoseWithCovarianceStamped()
        

        
    
    def demo_callback(self, msg):
        """_summary_

        Args:
            msg (VehicleOdometry): Vehicle odometry data. Fits ROS REP 147 for aerial vehicles.
                                   More INFO: https://docs.px4.io/main/en/msg_docs/VehicleOdometry.html
                                   

        """
        
        print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
        print("RECEIVED SENSOR COMBINED DATA")
        print("=============================")

        position_ned = msg.position
        q_ned_frd = msg.q
        q_frd_ned = mtf.qinverse(q_ned_frd)
        
        print("REFERENCE FRAME: ", msg.pose_frame)
        print("Original_position: ", position_ned)
        print("Original_orientation: ", msg.q)
        
    
        q_inv = mtf.qinverse(msg.q)
        q_out = mtf.px4_to_ros_orientation(msg.q)
        
        
        
        ##************************************************
        position_enu = mtf.transform_position(position_ned, "NED_2_ENU")
        #print("ENU Position: ", position_enu)
        #
        #q_flu_enu = mtf.transform_orientation(q_flu_ned, "NED_2_ENU")
        #q_flu_ned = mtf.transform_orientation(q_frd_ned, "FRD_2_FLU")
        #
        #q_enu_flu = mtf.qinverse(q_flu_enu)
        #print("FLU_ENU q: ", q_flu_enu)
        #print("ENU_FLU q: ", q_enu_flu)
        #
        pose_variance_ned = np.hstack((msg.position_variance, msg.orientation_variance))
        pose_covariance_ned = np.eye(6) * pose_variance_ned
        pose_covariance_enu = mtf.transform_cov6d(pose_covariance_ned, "NED_2_ENU")
        #print(pose_covariance_enu)
        ##************************************************
        

        #position_ned = mtf.transform_position(position_enu, "ENU_2_NED")
        #print("NED Position: ", position_ned)
        #
        #q_frd_enu = mtf.transform_orientation(q_flu_enu, "FLU_2_FRD")
        #q_frd_ned = mtf.transform_orientation(q_frd_enu, "ENU_2_NED")
        #q_enu_frd= mtf.qinverse(q_frd_enu)
        #print("FRD_NED q: ", q_frd_ned)
        
        """
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        msg.pose.pose.position.x
        msg.pose.pose.position.y
        msg.pose.pose.position.z
        
        msg.pose.pose.orientation.x
        msg.pose.pose.orientation.y
        msg.pose.pose.orientation.z
        msg.pose.pose.orientation.w
        
        msg.pose.covariance
        """
        
        self.pwcs.header.stamp = self.get_clock().now().to_msg()
        self.pwcs.header.frame_id = "drone"
        
        self.pwcs.pose.pose.position.x = float(position_enu[0])
        self.pwcs.pose.pose.position.y = float(position_enu[1])
        self.pwcs.pose.pose.position.z = float(position_enu[2])
        
        self.pwcs.pose.pose.orientation.w = float(q_out[0])
        self.pwcs.pose.pose.orientation.x = float(q_out[1]) 
        self.pwcs.pose.pose.orientation.y = float(q_out[2]) 
        self.pwcs.pose.pose.orientation.z = float(q_out[3])
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