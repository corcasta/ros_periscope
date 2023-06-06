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
        # first a pi/2 rotation around the Z-axis (down)
        qz = tf.quaternions.axangle2quat([0,0,1], np.pi/2)
        # then a pi rotation around the X-axis (old North/new East)
        qx = tf.quaternions.axangle2quat([1,0,0], np.pi)
        
        q = tf.quaternions.qmult(qx, qz)
        #print("DEBUG: ", q)
        #q_enu = tf.quaternions.qmult(q, q_ned)
        q_enu = tf.quaternions.qmult(q, q_ned)
        
        return q_enu
    
    def __qenu2ned(self, q_enu):
        # first a pi/2 rotation around the Z-axis (down)
        qz = tf.quaternions.axangle2quat([0,0,1], np.pi/2)
        # then a pi rotation around the X-axis (old North/new East)
        qx = tf.quaternions.axangle2quat([1,0,0], np.pi)
        
        q = tf.quaternions.qmult(qx, qz)
        #print("DEBUG: ", q)
        #q_enu = tf.quaternions.qmult(q, q_ned)
        q_ned = tf.quaternions.qmult(q, q_enu)

        return q_ned
    
    def demo_callback(self, msg):
        q_enu = self.__qned2enu(msg.q)
        position_enu = tf.quaternions.rotate_vector(msg.position, q_enu)
        
        
        print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
        print("RECEIVED SENSOR COMBINED DATA")
        print("=============================")
        #print("ts: ", msg.timestamp )
        #print("gyro_rad[0]: ", msg.gyro_rad[0])
        #print("gyro_rad[1]: ", msg.gyro_rad[1])
        #print("gyro_rad[2]: ", msg.gyro_rad[2])
        
        print("ENU Position: ", position_enu)
        print("ENU q: ", q_enu)
        
        q_ned = self.__qenu2ned(q_enu)
        position_ned = tf.quaternions.rotate_vector(position_enu, q_ned)
        print("NED Position: ", position_ned)
        print("NED q: ", q_ned)
        
        
        
        self.pwcs.pose.pose.position.x = msg.position[0]
        self.pwcs.pose.pose.position.y = msg.position[1]
        self.pwcs.pose.pose.position.z = msg.position[2]
        
        
        self.pwcs.pose.pose.orientation.x = msg.q[0]
        self.pwcs.pose.pose.orientation.y = msg.q[1] 
        self.pwcs.pose.pose.orientation.z = msg.q[2]
        self.pwcs.pose.pose.orientation.w = msg.q[3]
        
        self.publisher.publish(self.pwcs)
        
        
   
def main():
    rclpy.init()
    node = SensorCombinedListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()