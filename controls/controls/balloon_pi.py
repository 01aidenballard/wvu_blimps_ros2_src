
import rclpy
from rclpy.node import Node
from blimp_interfaces.msg import EscInput
from blimp_interfaces.msg import CameraCoord

class BalloonPI(Node):
	def __init__(self):
		self.kpx = 0.2
		#self.kdx = 0.03
		self.kix = 0.001
		
		self.kpy = 0.0 #0.205
		self.kiy = 0.0 #0.0005
		
		self.x_goal = 320
		self.y_goal = 240
		self.coord = [320,240]
		self.x_int_error = 0
		self.y_int_error = 0
		
		self.ESC_pin1 = 5
		self.ESC_pin2 = 6
		self.ESC_pin3 = 13
		self.ESC_pin4 = 26
		
		super().__init__("balloon_pi")
		self.subscriber = self.create_subscription(
			CameraCoord, "cam_data", self.callback_camera_data, 3
		)
		self.publisher = self.create_publisher(EscInput, "ESC_balloon_input", 10)
		self.create_timer(0.3, self.callback_pi_control_balloon)
		self.get_logger().info("Started pi control for balloon detection.")
	def callback_camera_data(self,msg):
		self.coord = msg.position

	def callback_pi_control_balloon(self):
		msg2 = EscInput()
		coord = self.coord
		self.x_error = self.x_goal - coord[0]

		self.y_error = self.y_goal - coord[1]

		self.x_int_error += self.x_error
		self.y_int_error += self.y_error

		LR_input = self.x_error*self.kpx + self.x_int_error*self.kix

		L_input = 1050.0
		R_input = 1050.0

		if LR_input < 0:
			L_input = 1100.0 + abs(LR_input)
			R_input = 1050.0
		elif LR_input > 0:
			R_input = 1100.0 + abs(LR_input)
			L_input = 1050.0

		UD_input = self.y_error*self.kpy + self.y_int_error*self.kiy
		D_input = 0.0
		U_input = 0.0

		if UD_input < 0:
			D_input = float(1100 + abs(UD_input))
			U_input = 1050.0
		elif UD_input > 0:
			U_input = float(1100 + abs(UD_input))
			D_input = 1050.0

		#string = "L M: " +str(L_input) + " R M: " + str(R_input) + " U M: " + str(U_input) + "D M: " + str(D_input)
		#self.get_logger().info(string)
		
		msg2.esc_pins = [self.ESC_pin1, self.ESC_pin2, self.ESC_pin3, self.ESC_pin4]
		msg2.esc_pwm = [L_input,R_input,U_input,D_input]
		#self.get_logger().info(str(coord))
		#self.get_logger().info("LR Input: " + str(LR_input) + " L Input: " + str(L_input) + " R Input: " + str(R_input) + " X: " + 
		#	str(coord[0]))
		self.publisher.publish(msg2)



def main(args=None):
	rclpy.init(args=args)
	node = BalloonPI()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
