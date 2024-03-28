#external Libraries
import rclpy
from rclpy.node import Node
import numpy as np
import math
import time

#Message interfaces
from blimp_interfaces.msg import ImuData
from blimp_interfaces.msg import CartCoord
from blimp_interfaces.msg import BaroData

# inertial frame of the balloon:
	# x-axis points to the front of balloon when the system is powered on or started
	# z-axis points to perpendicular to the ground going into the ground
	# y-axis is the cross product of those directions
	# this frame will always be fixed in the same orientation hence the name inertial frame
# Body frame of the balloon:
	# x-axis points out of the front of the balloon
	# y-axis points to the right side of the balloon if you are lookting down the x-axis
	# z-axis is the right hand rule or the cross product between x and y axis
# Note
# If the roll pitch yaw of the balloon equals zero to the relatve start. the body frame and inertial frame alighn

class DynamicModel(Node):
	def __init__(self):
		super().__init__("dynamic_model")

		self.height = 0.0

		self.subscriber_imu = self.create_subscription(ImuData, "imu_data", self.callback_imu, 10)

		self.subscriber_baro = self.create_subscription(BaroData,"barometer_data", self.callback_baro, 10)

		self.subscriber_dynammic_model = self.create_subscription(EscInput, "ESC_input", self.callback_dynamic_model, 10)

		self.get_logger().info("dynamics are modeled")

	def callback_baro(self,msg):
		self.height = msg.height

	def callback_imu(self,msg):
		self.euler = msg.imu_euler
		self.lin_accel = msg.imu_lin_accel
		self.gyro = msg.imu_gyro
		# rotate shit

	def callback_dynamic_model(self, msg):
		print("hello")
		






def main(args=None):
	rclpy.init(args=args)
	node = EscControl()
	rclpy.spin(node)
	os.system("sudo killall pigpiod")
	rclpy.shutdown()

if __name__ == "__main__":
	main()