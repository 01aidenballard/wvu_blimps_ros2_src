import rclpy
from rclpy.node import Node
from blimp_interfaces.msg import EscInput 
from blimp_interfaces.msg import CameraCoord
from scipy.optimize import lsq_linear
import numpy as np


class BomberNode(Node):
	def __init__(self):

		super().__init__("Bomber_node")
		
		self.declare_parameter('k',1) #kg/m^3
		self.k = self.get_parameter('k').value
		self.subscriber = self.create_subscription(
			CameraCoord, "cam_data", self.callback_bomber, 3
		)
		self.x_cntr = 320
		self.y_cntr = 240
		
		self.publisher = self.create_publisher(EscInput, "ESC_Bomber_input", 10)
		self.get_logger().info("Bombing civilians")

	def callback_bomber(self,msg):
		#pwm_l is motor A:
        #pwm_r is motor B:
		#pwm_d is motor C:
        # top of camera needs to be pointing towards A aka pwm_l
        #See documentation if scott ever gets to that gl brother.
		x = msg.position[0]
		y = msg.position[1]
		v = np.array([x-self.x_cntr, self.y_cntr - y, 0])
		
		M = np.array([[0,0.866025,-0.866025],[1,-0.5,-0.5],[0,0,0]])
		F = lsq_linear(M, v, bounds=(0, np.inf)).x
		msg2 = EscInput()
		#self.get_logger().info(str(F[0]))
		msg2.pwm_l = float(F[0])*self.k + 1050.0
		msg2.pwm_r = float(F[1])*self.k + 1050.0
		msg2.pwm_u = 1050.0
		msg2.pwm_d = float(F[2])*self.k + 1050.0
		msg2.esc_pins = [5,6,13,26]
		self.get_logger().info("A: " + str(msg2.pwm_l) + " B: " + str(msg2.pwm_r) + " C: " + str(msg2.pwm_d))
		self.publisher.publish(msg2)

def main(args=None):
	rclpy.init(args=args)
	node = BomberNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
