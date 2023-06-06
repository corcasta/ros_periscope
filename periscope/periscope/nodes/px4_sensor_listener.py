import rclpy
import numpy as np
import transforms3d as tf
from rclpy.node import Node
from px4_msgs.msg import SensorCombined, VehicleOdometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data
#import px4_ros_com.src.lib

class SensorCombinedListener(Node):
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
        
        
    def __qned2enu(self, q_ned):
        
        # Quaternion: [w, x, y, z]
        q_ned = [q_ned[-1], q_ned[0], q_ned[1], q_ned[2]]
        # Quaternion: [w, x, y, z]
        q = [0, np.sqrt(2)/2, np.sqrt(2)/2, 0]
        
        q_enu = tf.quaternions.qmult(q_ned, q)
        
        # Quaternion: [x, y, z, w]
        q_enu = [q_enu[-1], q_enu[0], q_enu[1], q_enu[2]]
        return q_enu
    
    def __qenu2ned(self, q_enu):
                
        # Quaternion: [w, x, y, z]
        q_enu = [q_enu[-1], q_enu[0], q_enu[1], q_enu[2]]
        
        # Quaternion: [w, x, y, z]
        q = [0, np.sqrt(2)/2, np.sqrt(2)/2, 0]
        
        q_ned = tf.quaternions.qmult(q_enu, q)
        
        # Quaternion: [x, y, z, w]
        q_ned = [q_ned[-1], q_ned[0], q_ned[1], q_ned[2]]
        return q_ned
    
    def demo_callback(self, msg):
        
        print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
        print("RECEIVED SENSOR COMBINED DATA")
        print("=============================")
        #print("ts: ", msg.timestamp )
        #print("gyro_rad[0]: ", msg.gyro_rad[0])
        #print("gyro_rad[1]: ", msg.gyro_rad[1])
        #print("gyro_rad[2]: ", msg.gyro_rad[2])
        
        q_enu = self.__qned2enu(msg.q)
        q_ned = self.__qenu2ned(q_enu)
        
        position_enu = tf.quaternions.rotate_vector(msg.position, [0, np.sqrt(2)/2, np.sqrt(2)/2, 0])
        print("ENU Position: ", position_enu)
        print("ENU q: ", q_enu)
        
        
        position_ned = tf.quaternions.rotate_vector(position_enu, [0, np.sqrt(2)/2, np.sqrt(2)/2, 0])
        print("NED Position: ", position_ned)
        print("NED q: ", q_ned)
        
        
        
        self.pwcs.pose.pose.position.x = float(position_enu[0])
        self.pwcs.pose.pose.position.y = float(position_enu[1])
        self.pwcs.pose.pose.position.z = float(position_enu[2])
        
        
        self.pwcs.pose.pose.orientation.x = float(q_enu[0])
        self.pwcs.pose.pose.orientation.y = float(q_enu[1]) 
        self.pwcs.pose.pose.orientation.z = float(q_enu[2])
        self.pwcs.pose.pose.orientation.w = float(q_enu[3])
        
        self.publisher.publish(self.pwcs)
        
        
   
def main():
    rclpy.init()
    node = SensorCombinedListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()