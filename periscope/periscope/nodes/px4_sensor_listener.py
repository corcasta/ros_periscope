import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from px4_msgs.msg import SensorCombined


class SensorCombinedListener(Node):
    """
    DEMO on How to interface PX4 data
    """
    def __init__(self):
        super().__init__("px4_sensors")

        print(type(qos_profile_sensor_data))
        self.subscription = self.create_subscription(SensorCombined,
                                                     "/fmu/out/sensor_combined",
                                                     self.demo_callback,
                                                     qos_profile_sensor_data)
          
    def demo_callback(self, msg):
        print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
        print("RECEIVED SENSOR COMBINED DATA")
        print("=============================")
        print("ts: ", msg.timestamp )
        print("gyro_rad[0]: ", msg.gyro_rad[0])
        print("gyro_rad[1]: ", msg.gyro_rad[1])
        print("gyro_rad[2]: ", msg.gyro_rad[2])
  
   
def main():
    rclpy.init()
    node = SensorCombinedListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()