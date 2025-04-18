import rclpy
from rclpy.node import Node
from blimp_interfaces.msg import EscInputRudolph 
from blimp_interfaces.msg import CameraCoord
from sensor_msgs.msg import Joy
import time

class Rudolph(Node):
	def __init__(self):
		super().__init__("mode_switcher")

		# Initial state
		self.Manual_mode = True

		# Manual motor data
		self.manual_pins = [5,6,13,26]
		self.manual_sm1 = 0
		self.manual_sm2 = 0
		self.manual_sm3 = 0
		self.manual_vm = 0

		# Auto motor data
		self.auto_pins = [5,6,13,26]
		self.auto_sm1 = 0
		self.auto_sm2 = 0
		self.auto_sm3 = 0
		self.auto_vm = 0

		# Subscriptions
		self.manual_subscriber = self.create_subscription(
			EscInputRudolph, "ESC_Manual_input", self.callback_manual, 10
		)

		self.baro_subscriber = self.create_subscription(
			EscInputRudolph, "ESC_Baro_input", self.callback_altitude_control, 10
		)

		self.subscriber = self.create_subscription(
			Joy, "joy", self.callback_switch_mode, 10
		)

		# Publisher to ESC driver
		self.publisher = self.create_publisher(EscInputRudolph, "ESC_input", 10)
		self.get_logger().info("Started altitude control for light house.")

	def callback_manual(self, msg):
		self.manual_pins = msg.esc_pins
		self.manual_sm1 = float(msg.pwm_sm1)
		self.manual_sm2 = float(msg.pwm_sm2)
		self.manual_sm3 = float(msg.pwm_sm3)
		self.manual_vm = float(msg.pwm_vm)

	def callback_altitude_control(self, msg):
		self.auto_pins = msg.esc_pins
		self.auto_sm1 = float(msg.pwm_sm1)
		self.auto_sm2 = float(msg.pwm_sm2)
		self.auto_sm3 = float(msg.pwm_sm3)
		self.auto_vm = float(msg.pwm_vm)

	def callback_switch_mode(self, msg):
		msg2 = EscInputRudolph()

		if msg.buttons[0] == 1:
			self.Manual_mode = not self.Manual_mode
			time.sleep(2)
			self.get_logger().info("Manual Mode is " + str(self.Manual_mode))

		if self.Manual_mode:
			msg2.esc_pins = self.manual_pins
			msg2.pwm_sm1 = float(self.manual_sm1)
			msg2.pwm_sm2 = float(self.manual_sm2)
			msg2.pwm_sm3 = float(self.manual_sm3)
			msg2.pwm_vm = float(self.manual_vm)
		else:
			msg2.esc_pins = self.auto_pins
			msg2.pwm_sm1 = float(self.auto_sm1)
			msg2.pwm_sm2 = float(self.auto_sm2)
			msg2.pwm_sm3 = float(self.auto_sm3)
			msg2.pwm_vm = float(self.auto_vm)

		self.publisher.publish(msg2)

def main(args=None):
	rclpy.init(args=args)
	node = Rudolph()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
