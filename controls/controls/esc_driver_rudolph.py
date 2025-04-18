import rclpy								# ros2 package for python
from rclpy.node import Node					# samsies
from blimp_interfaces.msg import EscInputRudolph  	# custom interfaces for Esc inputs
import pigpio								# gpio library
import os
import time
import subprocess as sp
os.system("sudo pigpiod")
time.sleep(1)

class EscControl(Node):
	def __init__(self):
		
		self.pi = pigpio.pi() #assigning self.pi as pigpio object
		#initializing variables
		self.pwm_sm1 = 0
		self.pwm_sm2 = 0
		self.pwm_sm3 = 0
		self.pwm_vm = 0

		super().__init__("esc_driver_rudolph") # initialing ros2 node
		#decalreing a parameter MAC to inport the mac adress
		self.declare_parameter("MAC","68:6C:E6:73:04:62")
		#getting the value of that parameter
		self.MAC = self.get_parameter("MAC").get_parameter_value().string_value
		#subscribing to the ESC_input topic
		self.subscriber = self.create_subscription(
			EscInputRudolph, "ESC_input", self.callback_control_the_esc, 10
		)

		#printing that the node has started
		self.get_logger().info("ESC is controlled")

	def callback_control_the_esc(self, msg):
		# assigning the variables from the message
		pins = msg.esc_pins

		#assign ing the msg to the global variable and checking the bounds of the signal
		self.pwm_sm1 = self.limit(msg.pwm_sm1)
		self.pwm_sm2 = self.limit(msg.pwm_sm2)
		self.pwm_sm3 = self.limit(msg.pwm_sm3)
		self.pwm_vm = self.limit(msg.pwm_vm)

		#grabing the bluetooth info
		stdoutdata = sp.getoutput('hcitool con')
		#checking if the MAC adress is not in the list
		# if no setting every value of the motors to zero
		#else input new values
		if not(self.MAC in stdoutdata.split()):
			print('Bluetooth Disconnected')
			self.pi.set_servo_pulsewidth(pins[0], 0)
			self.pi.set_servo_pulsewidth(pins[1], 0)
			self.pi.set_servo_pulsewidth(pins[2], 0)
			self.pi.set_servo_pulsewidth(pins[3], 0)
		else:
			self.pi.set_servo_pulsewidth(pins[0], self.pwm_sm1)
			self.pi.set_servo_pulsewidth(pins[1], self.pwm_sm2)
			self.pi.set_servo_pulsewidth(pins[2], self.pwm_sm3)
			self.pi.set_servo_pulsewidth(pins[3], self.pwm_vm)
	def limit(self,pwm):
		if pwm > 1950.0:
			pwm = 1950.0
		elif pwm < 1050.0:
			pwm = 1050.0
		return pwm








def main(args=None):
	rclpy.init(args=args)
	node = EscControl()
	rclpy.spin(node)
	os.system("sudo killall pigpiod")
	rclpy.shutdown()

if __name__ == "__main__":
	main()
