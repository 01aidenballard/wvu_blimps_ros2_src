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
from blimp_interfaces.msg import EscInput

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

		# Balloon Characteristics
		a = 0.86
		b = 0.28
		c = 0.38

		Ix = 0.058241
		Iy = 0.08583106
		Iz = 0.04598382
		self.m = 0.42511728
		self.zg = 0.21788162

		V_balloon = 0.41455956
		self.W = self.m *9.81
		self.declare_parameter('bouyancy',self.W)

		self.B = self.get_parameter('boyancy').value

		# Drag Coeeficients
		CD = 0.083136
		CL = 0.209683
		CY = 0.440926
		Cl = 0.0019314
		Cm = 0.212134
		Cn = 0.458387

		self.M_matrix(a, b, c, rho_air, Ix, Iy, Iz)
		self.D_matrix(CD, CY, CL, Cl, Cm, Cn, rho_air, V_balloon)

		self.vx = 5
		self.vy = 5
		self.vz = 5

		self.subscriber_imu = self.create_subscription(ImuData, "imu_data", self.callback_imu, 10)

		self.subscriber_baro = self.create_subscription(BaroData,"barometer_data", self.callback_baro, 10)

		self.subscriber_dynammic_model = self.create_subscription(EscInput, "ESC_input", self.callback_dynamic_model, 10)

		self.get_logger().info("dynamics are modeled")

	def M_matrix(self,a,b,c,rho,Ix,Iy,Iz):

		#ellipsode calc
		e = np.sqrt((1-(b/a)**2))
		beta0 = (1/(e**2)) - (np.log((1+e)/1-e) * ((1-e**2)/(2*e**3)))
		alpha0 = ((2*(1-e**2))/e**3) * ((0.5*(np.log((1+e)/(1-e))))-e)
		kprime = (e**4 * (beta0-alpha0)) / ((2-e**2)*(2*e**2 - (2-e**2) * (beta0-alpha0)))
		k1 = alpha0/(2-alpha0)
		k2 = beta0/(2-beta0)
		Izh = (4/5)*np.pi*rho*a*b**2*(a**2+b**2)
		self.mx_prime = self.m + k1*self.m
		self.my_prime = self.m + k2*self.m
		self.mz_prime = self.m + k2*self.m
		self.Ix_prime = Ix
		self.Iy_prime = Iy + kprime*Izh
		self.Iz_prime = Iz + kprime*Izh

		self.M = np.array([ (self.mx_prime,        0,        0,        0,     self.m*self.zg,        0),
					 		(       0, self.my_prime,        0,    -self.m*self.zg,        0,        0),
							(       0,        0, self.mz_prime,        0,        0,        0),
							(       0,    -self.m*self.zg,        0, self.Ix_prime,        0,        0),
							(    self.m*self.zg,        0,        0,        0, self.Iy_prime,        0),
							(       0,        0,        0,        0,        0, self.Iz_prime)])
		
	def D_matrix(self, CD, CY, CL, Cl, Cm, Cn, rho, V):
		Du = 0.5*CD*rho*V**(2/3)         # Damping Coefficients
		Dv = 0.5*CY*rho*V**(2/3)
		Dw = 0.5*CL*rho*V**(2/3)
		Dp = 0.5*Cl*rho*V
		Dq = 0.5*Cm*rho*V
		Dr = 0.5*Cn*rho*V 
		self.D = np.diag(np.array([(Du, Dv, Dw, Dp, Dq, Dr)]))

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
		
		self.C = np.array( [(0, 0, 0, 0, -(self.mz_prime*self.vz) -(self.my_prime*self.vy - self.m*self.zg*self.gyro[0])),
					 		(0, 0, 0, -(self.mz_prime*self.vz), 0, -(-self.mx_prime*self.vx - self.m*self.zg*self.gyro[1])),
							(0, 0, 0, -(-self.my_prime*self.vy - self.m*self.zg*self.gyro[0]), -(self.mx_prime*self.vx + self.m*self.zg*self.gyro[1]), 0),
							(0, -(-self.mz_prime*self.vz), -(self.my_prime*self.vy - self.m*self.zg*self.gyro[0]), 0, -(-self.Iz_prime*self.gyro[2]), -(self.m*self.zg*self.vx + self.Iy_prime*self.gyro[1])),
							(-(self.mz_prime*self.vz), 0, -(-self.mx_prime*self.vx - self.m*self.zg*self.gyro[1]), -(self.Iz_prime*self.gyro[2]), 0, -(self.m*self.zg*self.vy - self.Ix_prime*self.gyro[0])),
							(-(-self.my_prime*self.vy - self.m*self.zg*self.gyro[0]), -(self.mx_prime*self.vx + self.m*self.zg*self.gyro[1]), 0, -(-self.m*self.zg*self.vx - self.Iy_prime*self.gyro[1]), -(-self.m*self.zg*self.vy + self.Ix_prime*self.gyro[0]), 0)])
		
		self.g = np.array([ ((self.W-self.B)*np.sin(self.euler[1])),
					 		((self.W-self.B)*np.cos(self.euler[1])*np.sin(self.euler[0])),
							((self.W-self.B)*np.cos(self.euler[1])*np.cos(self.euler[0])),
							(-self.zg*self.W*np.cos(self.euler[1])*np.sin(self.euler[0])),
							(-self.zg*self.W*np.sin(self.euler[1])),
							(0)])
		






def main(args=None):
	rclpy.init(args=args)
	node = EscControl()
	rclpy.spin(node)
	os.system("sudo killall pigpiod")
	rclpy.shutdown()

if __name__ == "__main__":
	main()