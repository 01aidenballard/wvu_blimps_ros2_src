import rclpy
from rclpy.node import Node
from blimp_interfaces.msg import EscInput 
import time


class MuxNode(Node):
	def __init__(self):

		super().__init__("mux_node")
		self.bomber_a = 0.0
		self.bomber_b = 0.0
		self.bomber_c = 0.0
		self.bomber_esc =[0,0,0,0]
		self.random_a = 0.0
		self.random_b = 0.0
		self.random_c = 0.0
		self.random_esc = [0,0,0,0]
		self.time = 0.0
		self.baro_u = 0.0
		self.baro_esc = 0.0
		
		self.subscriber_bomber = self.create_subscription(EscInput, "ESC_Bomber_input", self.callback_bomber, 10)
		self.subscriber_random = self.create_subscription(EscInput, "ESC_random_input", self.callback_random, 10)
		self.subscriber_baro = self.create_subscription(EscInput, "ESC_Baro_input", self.callback_baro, 10)
		
		self.publisher = self.create_publisher(EscInput, "ESC_input", 10)
		
		self.create_timer(0.1,self.callback_mux)
		
		self.get_logger().info("combining inputs")

	def callback_bomber(self,msg):
		self.bomber_a = msg.pwm_l
		self.bomber_b = msg.pwm_r
		self.bomber_c = msg.pwm_d
		self.bomber_esc = msg.esc_pins
		self.time = time.time()
		
	def callback_random(self,msg):
		self.random_a = msg.pwm_l
		self.random_b = msg.pwm_r
		self.random_c = msg.pwm_d
		self.random_esc = msg.esc_pins
		
	def callback_baro(self,msg):
		self.baro_u = msg.pwm_u
		self.baro_esc = msg.esc_pins
		
	def callback_mux(self):
		dt = time.time() - self.time
		msg = EscInput()
		msg.pwm_u = self.baro_u
		if dt > 10:
			msg.pwm_l = self.random_a
			msg.pwm_r = self.random_b
			msg.pwm_d = self.random_c
		else:
			msg.pwm_l = self.bomber_a
			msg.pwm_r = self.bomber_b
			msg.pwm_d = self.bomber_c
		msg.esc_pins = [5,6,13,26]
		
		self.publisher.publish(msg)
	
	
		

def main(args=None):
	rclpy.init(args=args)
	node = MuxNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
