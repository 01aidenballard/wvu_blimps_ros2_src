# -*- coding: utf-8 -*-
import serial
import time
import rclpy
from rclpy.node import Node
from blimp_interfaces.msg import LidarData  # Assuming you have a similar message type defined

class SupersonicSensorNode(Node):
    def __init__(self):
        super().__init__('supersonic_sensor_node')
        self.lidar_data = self.create_publisher(LidarData, "lidar_data", 10)  # Adjust if message type changes
        self.ser = serial.Serial("/dev/ttyS0", 115200, timeout=1)  # Adjust serial port and settings
        self.create_timer(0.1, self.publish_supersonic_data)

    def publish_supersonic_data(self):
        msg = LidarData()  # Adjust message type if necessary
        if self.ser.in_waiting > 0:
            distance_str = self.ser.readline().strip().decode('utf-8')  # Read distance data as string
            try:
                msg.distance = float(distance_str)  # Convert distance string to float
                self.get_logger().info(f"Received distance: {msg.distance}")
                self.lidar_data.publish(msg)
            except ValueError as e:
                self.get_logger().error(f"Error converting distance: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SupersonicSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
