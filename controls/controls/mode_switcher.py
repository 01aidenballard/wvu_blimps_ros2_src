import rclpy
from rclpy.node import Node
from blimp_interfaces.msg import EscInput 
from blimp_interfaces.msg import CameraCoord
from sensor_msgs.msg import Joy
import time

class BalloonPI(Node):
	def __init__(self):
		self.Manual_mode = True

		self.manual_pins = [5,6,13,26]
		self.manual_L = 0
		self.manual_R = 0
		self.manual_U = 0
		self.manual_D = 0

		self.camera_pins = self.manual_pins
		self.camera_L = 0
		self.camera_R = 0
		self.camera_U = 0
		self.camera_D = 0

		super().__init__("mode_switcher")
				
		self.manual_subscriber = self.create_subscription(
			EscInput, "ESC_Manual_input", self.callback_manual, 10
		)
		
		self.camera_subscriber = self.create_subscription(
			EscInput, "ESC_balloon_input", self.callback_camera, 10
		)
		
		self.subscriber = self.create_subscription(
			Joy, "joy", self.callback_switch_mode, 10
		)
		
		self.publisher = self.create_publisher(EscInput, "ESC_input", 10)
				
		self.get_logger().info("Started pi control for balloon detection.")
		
	def callback_camera(self,msg):
		self.camera_pins = msg.esc_pins
		self.camera_L = float(msg.pwm_l)
		self.camera_R = float(msg.pwm_r)
		self.camera_U = float(msg.pwm_u)
		self.camera_D = float(msg.pwm_d)
		
	def callback_manual(self,msg):
		self.manual_pins = msg.esc_pins
		self.manual_L = float(msg.pwm_l)
		self.manual_R = float(msg.pwm_r)
		self.manual_U = float(msg.pwm_u)
		self.manual_D = float(msg.pwm_d)
		

	def callback_switch_mode(self, msg):
		msg2 = EscInput()
		
		if msg.buttons[0]  == 1:
			self.Manual_mode = not self.Manual_mode
			time.sleep(2)
			self.get_logger().info("Manual Mode is " + str(self.Manual_mode))
			
		
		if self.Manual_mode == True:
			msg2.esc_pins = self.manual_pins
			msg2.pwm_l = float(self.manual_L)
			msg2.pwm_r = float(self.manual_R)
			msg2.pwm_u = float(self.manual_U)
			msg2.pwm_d = float(self.manual_D)
		
		elif self.Manual_mode == False:
			msg2.esc_pins = self.camera_pins
			msg2.pwm_l = float(self.camera_L)
			msg2.pwm_r = float(self.camera_R)
			msg2.pwm_u = float(self.camera_U)
			msg2.pwm_d = float(self.camera_D)
		
		self.publisher.publish(msg2)
			
		


def main(args=None):
	rclpy.init(args=args)
	node = BalloonPI()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
