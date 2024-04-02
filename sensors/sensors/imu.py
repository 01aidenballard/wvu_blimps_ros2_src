  
import rclpy
from rclpy.node import Node
import time
import board
import busio
import adafruit_bno055
import numpy as np
import math
from blimp_interfaces.msg import ImuData


class ImuNode(Node): #Creating a Node

	def __init__(self): #initiating node
		super().__init__('imu_node') #naming node 'imu_node'
        # The BNO055 setup here is for i2c protocol. If you want to start using uart, you need to look it up.
        
		self.imu_data = self.create_publisher(ImuData,"imu_data",10) #Initializing publisher (message type,name,Qsize(some buffer thing:10 messages before it erases last one)S)
		self.i2c = busio.I2C(board.SCL, board.SDA)
		self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
		self.sensor.mode = adafruit_bno055.M4G_MODE
		self.sensor.mode = adafruit_bno055.NDOF_MODE
		self.create_timer(0.2, self.publish_imu_data) #calls function every 0.2 seconds
		# initializing lists of saved variables and send variables
		self.prev_lin_accel = [0 ,0 ,0]
		self.prev_gyro = [0 ,0 ,0]
		self.prev_euler = [0 ,0 ,0]
		self.imu_lin_accel = [0 ,0 ,0]
		self.imu_gyro = [0 ,0 ,0]
		self.imu_euler = [0 ,0 ,0]

		self.start_time = time.time()

	def publish_imu_data(self):
		msg = ImuData()
		lin_accel = self.sensor.linear_acceleration
		gyro = self.sensor.gyro
		euler = self.sensor.euler
		#Check accelerations for NaNs
		for i in enumerate(lin_accel):
			if math.isnan(i[1]):
				self.imu_lin_accel[i[0]] = self.prev_lin_accel[i[0]]
			else:
				self.imu_lin_accel[i[0]] = lin_accel(i[0])
				self.prev_lin_accel[i[0]] = lin_accel(i[0])
		#Check gyros for NaNs
		for i in enumerate(gyro):
			if math.isnan(i[1]):
				self.imu_gyro[i[0]] = self.prev_gyro[i[0]]
			else:
				self.imu_gyro[i[0]] = gyro(i[0])
				self.prev_gyro[i[0]] = gyro(i[0])
		#Check Eulers for NaNs
		for i in enumerate(euler):
			if math.isnan(i[1]):
				self.imu_euler[i[0]] = self.prev_euler[i[0]]
			else:
				self.imu_euler[i[0]] = euler(i[0])
				self.prev_euler[i[0]] = euler(i[0])
		
		msg.imu_lin_accel = self.imu_lin_accel
		msg.imu_gyro = self.imu_gyro
		msg.imu_euler = self.imu_euler
	#	self.get_logger().info(str(msg.imu_euler)) # Displays data on command line
		self.imu_data.publish(msg)
        
def main(args=None):
	rclpy.init(args=args)
	node = ImuNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__': #this allows us to run script from terminal directly
	main()


