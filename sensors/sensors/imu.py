
import rclpy
from rclpy.node import Node
import time
import board
import busio
import adafruit_bno055
import numpy as np
import math
#from std_msgs.msg import Float32MultiArray #also have to add dependcy package into package.xml
from blimp_interfaces.msg import ImuData


class ImuNode(Node): #Creating a Node
    
    def __init__(self): #initiating node
        super().__init__('imu_node') #naming node 'imu_node'
        # If you are going to use UART uncomment these lines
        # self.uart = self.board.UART()
        # self.sensor = adafruit_bno055.BNO055_UART(uart)
        
        self.imu_data = self.create_publisher(ImuData,"imu_data",10) #Initializing publisher (message type,name,Qsize(some buffer thing:10 messages before it erases last one)S)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        self.sensor.mode = adafruit_bno055.M4G_MODE
        self.sensor.mode = adafruit_bno055.NDOF_MODE
        self.create_timer(0.2, self.publish_imu_data) #calls function every 0.2 seconds
        
    def publish_imu_data(self): 
        #linear_acceleration = self.sensor.get_linear_acceleration
        #msg.linear_acceleration.x = linear_acceleration[0]
        #self.send_imu_command_pub_.publish(msg)
        while True:
            #M_yaw = math.atan2(-sensor.magnetic[0],sensor.magnetic[1])
            #print("Temperature: {}".format(sensor.temperature))
            #print("Euler Angles: {}".format(sensor.euler)) #prints (Heading,Roll,Pitch)
            #print("Quaternion: {}".format(sensor.quaternion))
            #print("Temperature: {}".format(sensor.temperature))
            #print("Magnetometer: {}".format(sensor.magnetic))
            #msg = Imu()
            #msg.data = sensor.eulerself.publisher_.publish(msg)
            msg = ImuData()
            msg.imu_lin_accel = [self.sensor.linear_acceleration]
            msg.imu_gyro = [self.sensor.gyro]
            msg.imu_euler = [self.sensor.euler]
            #self.get_logger().info(str(msg.imu_lin_accel)) # Displays data on command line
            self.imu_data.publish(msg)

            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': #this allows us to run script from terminal directly
    main()


