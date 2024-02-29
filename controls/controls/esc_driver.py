import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from blimp_interfaces.msg import EscInput 
import pigpio
import os
import time
os.system("sudo pigpiod")
time.sleep(1)

class EscControl(Node):
	def __init__(self):
		
		self.pi = pigpio.pi()

		super().__init__("manual_esc_control")
		self.subscriber = self.create_subscription(
			EscInput, "ESC_input", self.callback_control_the_esc, 10
		)
		
		self.subscriber = self.create_subscription(
			Float64, "time", self.callback_time, 10
		)

		self.get_logger().info("ESC is controlled")

	def callback_control_the_esc(self, msg):
		pins = msg.esc_pins
		pwm = msg.esc_pwm
		self.inputs = pwm
		# ~ self.get_logger().info(str(pwm[0]))

		self.pi.set_servo_pulsewidth(pins[0], pwm[0])
		self.pi.set_servo_pulsewidth(pins[1], pwm[1])
		self.pi.set_servo_pulsewidth(pins[2], pwm[2])
		self.pi.set_servo_pulsewidth(pins[3], pwm[3])
		
	def callback_time(self, msg):
		time1 = msg.data
		time2 = time.time()
		
		dtime = time2 - time1
		self.get_logger().info("Left Motor: " + str(self.inputs[0]) + " Right Motor: " + str(self.inputs[1]) + 
		" Up Motor: " + str(self.inputs[2]) + " Down Motor: " + str(self.inputs[3]) + " Time:" + str(dtime))
		
		f = open("time_latency_test6.txt", 'a')
		f.write("Left Motor: " + str(self.inputs[0]) + " Right Motor: " + str(self.inputs[1]) + 
		" Up Motor: " + str(self.inputs[2]) + " Down Motor: " + str(self.inputs[3]) + " Time:" + str(dtime) + "\n")
		
		





def main(args=None):
	rclpy.init(args=args)
	node = EscControl()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
