import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from blimp_interfaces.msg import EscInput
from std_msgs.msg import String 
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
import time

class FixAxesNode(Node):
	def __init__(self):
		
		self.ESC_pin1 = 5
		self.ESC_pin2 = 6
		self.ESC_pin3 = 13
		self.ESC_pin4 = 26

		super().__init__("esc_input")
		
		self.subscriber = self.create_subscription(
			Joy, "joy", self.callback_manual_esc_input, 10
		)
		
		self.publisher = self.create_publisher(EscInput, "ESC_Manual_input", 10)		
		
		self.get_logger().info("Data is being sent to the ESC node")


	def callback_manual_esc_input(self, msg):
		
		RM = msg.axes[5]*-100
		LM = ((msg.axes[3]-1)*-100)/2
		if msg.axes[1] > 0.05:
			UM = abs(msg.axes[1])*100
			DM = 0
		elif msg.axes[1] < -0.05:
			DM = abs(msg.axes[1])*100
			UM = 0
		else:
			UM = 0
			DM = 0
		
		LM_pwm = self.control_to_esc_input(LM)
		UM_pwm = self.control_to_esc_input(UM)
		DM_pwm = self.control_to_esc_input(DM)
		RM_pwm = self.control_to_esc_input(RM)
		
		msg2 = EscInput()
		msg2.esc_pins = [self.ESC_pin1, self.ESC_pin2, self.ESC_pin3, self.ESC_pin4]
		msg2.esc_pwm = [LM_pwm,RM_pwm,UM_pwm,DM_pwm]

		self.publisher.publish(msg2)
		time.sleep(0.1)

		
	def control_to_esc_input(self, input):
		pwm = 1050 + (((input-0)*(1900-1050))/(100 - 0))

		return pwm

def main(args=None):
	rclpy.init(args=args)
	node = FixAxesNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
