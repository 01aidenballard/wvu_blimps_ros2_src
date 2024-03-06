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
		self.manual_pwm = [1100.0,1100.0,1100.0,1100.0]
		self.camera_pins = self.manual_pins
		self.camera_pwm = self. manual_pwm
		super().__init__("mode_switcher")
				
		self.subscriber = self.create_subscription(
			EscInput, "ESC_Manual_input", self.callback_manual, 10
		)
		
		self.subscriber = self.create_subscription(
			EscInput, "ESC_balloon_input", self.callback_camera, 10
		)
		
		self.subscriber = self.create_subscription(
			Joy, "joy", self.callback_switch_mode, 10
		)
		
		self.publisher = self.create_publisher(EscInput, "ESC_input", 10)
				
		self.get_logger().info("Started pi control for balloon detection.")
		
	def callback_camera(self,msg):
		self.camera_pins = msg.esc_pins
		self.camera_pwm = msg.esc_pwm
		self.get_logger().info("Camera")
	def callback_manual(self,msg):
		self.manual_pins = msg.esc_pins
		self.manual_pwm = msg.esc_pwm
		self.get_logger().info("Manual")

	def callback_switch_mode(self, msg):
		msg2 = EscInput()
		
		if msg.buttons[0]  == 1:
			self.Manual_mode is not self.Manual_mode
			self.get_logger().info("We are switching modes")
			time.sleep(2)
		
		if self.Manual_mode == True:
			msg2.esc_pins = self.manual_pins
			msg2.esc_pwm = self.manual_pwm
		
		elif self.Manual_mode == False:
			msg2.esc_pins = self.camera_pins
			msg2.esc_pwm = self.camera_pwm
		
		self.publisher.publish(msg2)
			
		


def main(args=None):
	rclpy.init(args=args)
	node = BalloonPI()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
