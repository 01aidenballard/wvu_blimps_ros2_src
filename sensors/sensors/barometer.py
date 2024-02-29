import rclpy
from rclpy.node import Node
import time
import board
import adafruit_bmp3xx # libraries for barometer, adafruit_bmp3xx
from std_msgs.msg import Float32MultiArray

class BarometerNode(Node): #Creating a Node

	def __init__(self):
		super().__init__('barometer_node')
		self.barometer_data = self.create_publisher(Float32MultiArray,"barometer_data",10)
		self.create_timer(0.2, self.publish_barometer_data)
		i2c = board.I2C() 
		sensor = adafruit_bmp3xx.BMP3XX_I2C(i2c)
		sensor.sea_level_pressure = 1019 #hPa, in Morgantown
		
	def publish_barometer_data(self):
		
		while True:
			msg = Float32MultiArray()
			msg.data = sensor.altitude
			self.barometer_data.publish(msg)
		
			time.sleep(1)
def main(args=None):
	rclpy.ini(args=args)
	node = BarometerNode()
	rclpy.spin(node)
	rclpy.shutdown()
	
if __name__ == '__main__': #this allows us to run script from terminal directly
	main()
		
