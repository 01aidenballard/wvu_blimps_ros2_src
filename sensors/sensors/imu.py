  
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
        self.start_time = time.time()
        
    def publish_imu_data(self):
    #def imu_read(self):    
        i2c = board.I2C()  # uses board.SCL and board.SDA
        # i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
        sensor = adafruit_bno055.BNO055_I2C(i2c)
        sensor.mode=adafruit_bno055.M4G_MODE
        #time.sleep(2)
        sensor.mode=adafruit_bno055.NDOF_MODE
        # If you are going to use UART uncomment these lines
        # uart = board.UART()
        # sensor = adafruit_bno055.BNO055_UART(uart)
        #linear_acceleration = self.BNO055_I2C.get_linear_acceleration
        #linear_acceleration = self.sensor.get_linear_acceleration
        #msg.linear_acceleration.x = linear_acceleration[0]
        #self.send_imu_command_pub_.publish(msg)
        #M_yaw = math.atan2(-sensor.magnetic[0],sensor.magnetic[1])
        #print("Temperature: {}".format(sensor.temperature))
        #print("Euler Angles: {}".format(sensor.euler)) #prints (Heading,Roll,Pitch)
        #print("Quaternion: {}".format(sensor.quaternion))
        #print("Temperature: {}".format(sensor.temperature))
        #print("Magnetometer: {}".format(sensor.magnetic))
        #msg = Imu()
        #msg.data = sensor.eulerself.publisher_.publish(msg)
        msg = ImuData()
        lin_accel = self.sensor.linear_acceleration
        gyro = self.sensor.gyro
        euler = self.sensor.euler
        msg.imu_lin_accel = [lin_accel[0],lin_accel[1],lin_accel[2]]
        msg.imu_gyro = [gyro[0],gyro[1],gyro[2]]
        msg.imu_euler = [euler[0],euler[1],euler[2]]
        #self.get_logger().info(str(msg.imu_euler)) # Displays data on command line
        self.imu_data.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': #this allows us to run script from terminal directly
    main()


