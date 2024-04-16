import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pigpio
import os
import time
os.system("sudo pigpiod")
time.sleep(1)

class NetServo(Node):
	def __init__(self):
		super().__init__("net_servo")
		self.pi = pigpio.pi()

		self.pin_net = 17
		self.net = False

		self.subscriber = self.create_subscription(
			Joy, "joy", self.callback_net, 10
		)
		self.get_logger().info("jimmies are ruffles")
	def callback_net(self, msg):
		
		if msg.buttons[1]  == 1:
			self.net = not self.net
			time.sleep(1)
			self.get_logger().info("Net closed is " + str(self.net))
		
		if self.net is False:
			self.pi.set_servo_pulsewidth(self.pin_net, 500)
		elif self.net is True:
			self.pi.set_servo_pulsewidth(self.pin_net, 2500)



def main(args=None):
	rclpy.init(args=args)
	node = NetServo()
	rclpy.spin(node)
	os.system("sudo killall pigpiod")
	rclpy.shutdown()

if __name__ == "__main__":
	main()
