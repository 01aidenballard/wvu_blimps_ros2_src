import rclpy						#ros2 package for python
from rclpy.node import Node			#samsies
from sensor_msgs.msg import Joy		#interface that the controller script publishes with
import pigpio						#library for the gpio pins
import os							#allows for usage of cmd line inputs in the python script
import time							#time

#starting the pigpio daemon. need the sleep after because it needs time to process
os.system("sudo pigpiod")
time.sleep(1)


class NetServo(Node):
	def __init__(self):
		super().__init__("net_servo") #initializing the node with the name net_servo

		#assigning the class object to self.pi
		self.pi = pigpio.pi()

		#the pin that the servo will use for pwm signal
		self.pin_net = 17
		# starting pos of net will be open
		self.net = False

		#subscribing to the /joy topic to get the button inputs 
		self.subscriber = self.create_subscription(
			Joy, "joy", self.callback_net, 10
		)

		#getlogger to show node has started
		self.get_logger().info("Net servo has started")


	def callback_net(self, msg):
		
		# if statement will check if the B button has been pressed
		if msg.buttons[1]  == 1:
			#changes the boolian to the opposit of itself
			self.net = not self.net
			#sleep to give you some time to unpress the button
			time.sleep(1)
			#showing that the state that the net was changed to
			self.get_logger().info("Net closed is " + str(self.net))
		
		# depending on the state of the net it will either open or close
		# False is open True is closed
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
