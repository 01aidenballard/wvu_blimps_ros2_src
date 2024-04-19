import rclpy
from rclpy.node import Node
from blimp_interfaces.msg import BaroData
from blimp_interfaces.msg import EscInput

class BaroNode(Node):
	def __init__(self):

		super().__init__("Baro_node")
		
		self.declare_parameter('kpb',0.0) #kg/m^3
		self.kpb = self.get_parameter('kpb').value
		self.declare_parameter('height',0.0) #kg/m^3
		self.height_goal = self.get_parameter('height').value
		self.subscriber = self.create_subscription(BaroData,"barometer_data", self.callback_baro_control, 10)
		self.publisher = self.create_publisher(EscInput, "ESC_Baro_input", 10)
		
		self.get_logger().info("barometer control has started")

	def callback_baro_control(self,msg):
		height = msg.height
		msg2 = EscInput()
		if height < self.height_goal:
			msg2.pwm_l = 1050.0
			msg2.pwm_r = 1050.0
			msg2.pwm_u = 1050 + abs(height-self.height_goal)*self.kpb
			msg2.pwm_d = 1050.0
		else:
			msg2.pwm_l = 1050.0
			msg2.pwm_r = 1050.0
			msg2.pwm_u = 1050.0
			msg2.pwm_d = 1050.0
		msg2.esc_pins = [5,6,13,26]
		self.publisher.publish(msg2)
		#self.get_logger().info("baro: " + str(height)+ " U: "+ str(msg2.pwm_u))
		
		
def main(args=None):
	rclpy.init(args=args)
	node = BaroNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
