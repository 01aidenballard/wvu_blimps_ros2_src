import rclpy
from rclpy.node import Node
#from std_msgs.msg import Float64
from blimp_interfaces.msg import EscInput 
from sensor_msgs.msg import Joy
import pigpio
import os
import time
os.system("sudo pigpiod")
time.sleep(1)

class EscControl(Node):
	def __init__(self):
		
		self.pi = pigpio.pi()
		self.pwm_L = 0
		self.pwm_R = 0
		self.pwm_U = 0
		self.pwm_D = 0
		self.pin_net = 17
		self.net = False

		super().__init__("manual_esc_control")
		self.subscriber = self.create_subscription(
			EscInput, "ESC_input", self.callback_control_the_esc, 10
		)

		self.get_logger().info("ESC is controlled")

	def callback_control_the_esc(self, msg):
		pins = msg.esc_pins
		self.pwm_L = msg.pwm_l
		self.pwm_R = msg.pwm_r
		self.pwm_U = msg.pwm_u
		self.pwm_D = msg.pwm_d
		#self.get_logger().info(str(pwm))

		self.pi.set_servo_pulsewidth(pins[0], self.pwm_L)
		self.pi.set_servo_pulsewidth(pins[1], self.pwm_R)
		self.pi.set_servo_pulsewidth(pins[2], self.pwm_U)
		self.pi.set_servo_pulsewidth(pins[3], self.pwm_D)





def main(args=None):
	rclpy.init(args=args)
	node = EscControl()
	rclpy.spin(node)
	os.system("sudo killall pigpiod")
	rclpy.shutdown()

if __name__ == "__main__":
	main()
