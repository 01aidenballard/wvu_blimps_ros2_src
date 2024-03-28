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
		self.declare_parameter('rho_air',1.225) #kg/m^3

		self.height = 0.0
		self.R_imu = np.array([(1, 0, 0),(0, -1, 0),(0, 0, -1)])
		rho_air = self.get_parameter('rho_air').value

		self.mass_matrix(0.86, 0.28, 0.38, rho_air, 0.05848714, 0.10314659, 0.06329918, 0.42511728, 0.21788162)

		self.subscriber_imu = self.create_subscription(ImuData, "imu_data", self.callback_imu, 10)

		self.subscriber_baro = self.create_subscription(BaroData,"barometer_data", self.callback_baro, 10)

		self.subscriber_dynammic_model = self.create_subscription(EscInput, "ESC_input", self.callback_dynamic_model, 10)

		self.get_logger().info("dynamics are modeled")

	def mass_matrix(self,a,b,c,rho,Ix,Iy,Iz,m,zg):
		#ellipsode calc
		e = np.sqrt((1-(b/a)**2))
		beta0 = (1/(e**2)) - (np.log((1+e)/1-e) * ((1-e**2)/(2*e**3)))
		alpha0 = ((2*(1-e**2))/e**3) * ((0.5*(np.log((1+e)/(1-e))))-e)
		kprime = (e**4 * (beta0-alpha0)) / ((2-e**2)*(2*e**2 - (2-e**2) * (beta0-alpha0)))
		k1 = alpha0/(2-alpha0)
		k2 = beta0/(2-beta0)
		Izh = (4/5)*np.pi*rho*a*b**2*(a**2+b**2)
		max = k1*m
		may = k2*m
		maz = k2*m
		Iax = 0
		Iay = kprime*Izh
		Iaz = kprime*Izh
		mx_prime = m + max
		my_prime = m + may
		mz_prime = m + maz
		Ix_prime = Ix +Iax
		Iy_prime = Iy + Iay
		Iz_prime = Iz + Iaz

		self.M = np.array([ (mx_prime, 0, 0, 0, m*zg, 0),
					 		(0, my_prime, 0,-m*zg, 0, 0),
							()])

	def callback_baro(self,msg):
		self.height = msg.height

	def callback_imu(self,msg):
		self.euler = np.array([(msg.imu_euler[0]),(msg.imu_euler[1]),(msg.imu_euler[2])])
		self.lin_accel = np.array([(msg.imu_lin_accel[0]),(msg.imu_lin_accel[1]),(msg.imu_lin_accel[2])])
		self.gyro = np.array([(msg.imu_gyro[0]),(msg.imu_gyro[1]),(msg.imu_gyro[2])])
		#rotate shit to match 
		self.euler = self.R_imu * self.euler
		self.lin_accel = self.R_imu *self.lin_accel
		self.gryo = self.R_imu * self.gyro

	def callback_dynamic_model(self, msg):
		
		






def main(args=None):
	rclpy.init(args=args)
	node = EscControl()
	rclpy.spin(node)
	os.system("sudo killall pigpiod")
	rclpy.shutdown()

if __name__ == "__main__":
	main()