  
import rclpy
from rclpy.node import Node
import time
import board
import adafruit_bno055
import numpy as np
import math
from std_msgs.msg import Float32MultiArray #also have to add dependcy package into package.xml


class ImuNode(Node): #Creating a Node
    
    def __init__(self): #initiating node
        super().__init__('imu_node') #naming node 'imu_node'
        self.imu_data = self.create_publisher(Float32MultiArray,"imu_data",10) #Initializing publisher (message type,name,Qsize(some buffer thing:10 messages before it erases last one)S)
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
            print(sensor.euler)
            msg = Float32MultiArray()
            msg.data = sensor.euler
            self.imu_data.publish(msg)
            self.record_imu(sensor)
            time.sleep(0.1)
    def record_imu(self,sensor):
        dt = time.time() - self.start_time
        f = open("imu_data_1.txt", 'a')
        f.write("Euler Angles: {}".format(sensor.euler) + " Lin Acceleration: {}".format(sensor.acceleration) +
         " Gyro: {}".format(sensor.gyro) + " Lin Acceleration NoG: {}".format(sensor.linear_acceleration) + " Gravity: {}".format(sensor.gravity) + " Time: {}".format(dt) + "\n")

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': #this allows us to run script from terminal directly
    main()


