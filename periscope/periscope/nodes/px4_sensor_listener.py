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
        
    def __transform_orientation(self, q, local="FLU_2_FRD", world="NED_2_ENU"):
        """
        BODY:
            FLU_2_FRD
            FRD_2_FLU
            SAME
        
        WORLD:
            NED_2_ENU
            ENU_2_NED
            SAME
        """

        # Quaternion: [w, x, y, z] 
        q = np.array([q[-1], q[0], q[1], q[2]])
        
        switch = local + "_&_" + world
        match switch:
            case "FRD_2_FLU_&_NED_2_ENU":
                # Quaternion: [w, x, y, z] 
                #q_new = tf.quaternions.qmult(q, [0, np.sqrt(2)/2, np.sqrt(2)/2, 0])
                #q_new = tf.quaternions.qmult([0, 1, 0, 0], q_new)
                
                
                r_1 = np.array([[1,  0,  0],
                                [0, -1,  0],
                                [0,  0, -1]]) 
                
                r_2 = np.array([[0,  1,  0],
                                [1,  0,  0],
                                [0,  0, -1]]) 
                
                r_main = tf.quaternions.quat2mat(q)
                
                r_final = r_1 @ r_main @ r_2
                
                q_final = tf.quaternions.mat2quat(r_final)
                q_final = [q_final[-1], q_final[0], q_final[1], q_final[2]]
                return q_final
            
            case "FLU_2_FRD_&_ENU_2_NED":
                # Quaternion: [w, x, y, z] 
                #q_new = tf.quaternions.qmult(q, [0, np.sqrt(2)/2, np.sqrt(2)/2, 0])
                #q_new = tf.quaternions.qmult([0, 1, 0, 0], q_new)
                
                
                r_1 = np.array([[1,  0,  0],
                                [0, -1,  0],
                                [0,  0, -1]]) 
                
                r_2 = np.array([[0,  1,  0],
                                [1,  0,  0],
                                [0,  0, -1]]) 
                
                r_main = tf.quaternions.quat2mat(q)
                
                r_final = r_1 @ r_main @ r_2
                
                q_final = tf.quaternions.mat2quat(r_final)
                q_final = [q_final[-1], q_final[0], q_final[1], q_final[2]]
                return q_final
                
            case "SAME_&_NED_2_ENU":
                # Quaternion: [w, x, y, z] 
                q_new = tf.quaternions.qmult([1, 0, 0, 0], q)
                q_new = tf.quaternions.qmult(q_new, [0, np.sqrt(2)/2, np.sqrt(2)/2, 0])
                
            case "SAME_&_ENU_2_NED":
                # Quaternion: [w, x, y, z] 
                q_new = tf.quaternions.qmult([1, 0, 0, 0], q)
                q_new = tf.quaternions.qmult(q_new, [0, np.sqrt(2)/2, np.sqrt(2)/2, 0])
                
            case "FRD_2_FLU_&_SAME":
                # Quaternion: [w, x, y, z] 
                q_new = tf.quaternions.qmult([0, 1, 0, 0], q)
                q_new = tf.quaternions.qmult(q_new, [1, 0, 0, 0])
                
            case "FLU_2_FRD_&_SAME":
                # Quaternion: [w, x, y, z] 
                q_new = tf.quaternions.qmult([0, 1, 0, 0], q)
                q_new = tf.quaternions.qmult(q_new, [1, 0, 0, 0])
        
        q_new = [q_new[-1], q_new[0], q_new[1], q_new[2]]    
        return q_new
        
        
    def __transform(self, q1):
        
        # Quaternion: [w, x, y, z]
        q1 = [q1[-1], q1[0], q1[1], q1[2]]
        
        # Quaternion: [w, x, y, z]
        q = [0, np.sqrt(2)/2, np.sqrt(2)/2, 0]
        
        rot = tf.quaternions.quat2mat(q)
        
        rot_t = tf.quaternions.quat2mat(q1)
        print("DEBUG: \n", rot_t)
        
        r = rot_t @ rot_t
        print("DEBUG2: \n", r)
        q_t = tf.quaternions.mat2quat(r)
        
        # Quaternion: [x, y, z, w]
        q_t = [q_t[-1], q_t[0], q_t[1], q_t[2]]
        
        return q_t
        #q = tf.euler.euler2quat(np.pi, 0, np.pi/2, 'rxyz')
        
        
        """
        print("DEBUG: ", q)
        # Extract the values from Q0
        x0 = q[0]
        y0 = q[1]
        z0 = q[2]
        w0 = q[3]
        
        # Extract the values from Q1
        x1 = q1[0]
        y1 = q1[1]
        z1 = q1[2]
        w1 = q1[3]
        
        # Computer the product of the two quaternions, term by term
        q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
        q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
        q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
        q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
        
        q_enu = [q1_x,
                 q1_y,
                 q1_z,
                 q1_w]      
          
        #q_enu = tf.quaternions.qmult(q, q_ned)
        
        # Quaternion: [x, y, z, w]
        #q_enu = [q_enu[-1], q_enu[0], q_enu[1], q_enu[2]]
        
        return q_enu
        """
        
        """
        Q1 = [0, np.sqrt(2)/2, np.sqrt(2)/2, 0]
        
        # Extract the values from Q0
        w0 = Q1[0]
        x0 = Q1[1]
        y0 = Q1[2]
        z0 = Q1[3]
        
        # Extract the values from Q1
        w1 = Q0[3]
        x1 = Q0[0]
        y1 = Q0[1]
        z1 = Q0[2]
        
        # Computer the product of the two quaternions, term by term
        Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
        Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
        Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
        Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

        Q0Q1 = [Q0Q1_x,
                Q0Q1_y,
                Q0Q1_z,
                Q0Q1_w]
        return Q0Q1
        """
    
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
        
        q_enu = self.__transform_orientation(msg.q, local="FRD_2_FLU", world="NED_2_ENU")
        q_ned = self.__transform_orientation(q_enu, local="FLU_2_FRD", world="ENU_2_NED")
        
        initial_position = np.array(msg.position)
        print("Original_position: ", initial_position)
        print("Original_orientation: ", msg.q)
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