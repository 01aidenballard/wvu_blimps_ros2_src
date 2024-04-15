# sender_node.py
import rclpy
from rclpy.node import Node
from blimp_interfaces.msg import EscInput
from blimp_interfaces.msg import UtcTime
import socket
import struct
import time


class SenderNode(Node):  # Creating a Node
    
    def __init__(self):  # initiating node
        super().__init__('sender_node')  # naming node 'sender_node'
        self.server_ip = '192.168.0.245'  # Replace with the receiver Pi's IP
        self.server_port = 12345
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.joy_time = 0.0
        self.subscriber_esc = self.create_subscription(
            EscInput, "ESC_input", self.callback_sender, 10)
        self.subscriber_time = self.create_subscription(
            UtcTime, "Time", self.callback_time, 10)
        self.get_logger().info("Data is sent")
        
    def callback_time(self, msg):
        self.joy_time = 0.0

    def callback_sender(self, msg):
        joy_time = 0.0
        send_time = time.time()
        float_vars = (msg.pwm_l, msg.pwm_r, msg.pwm_u, msg.pwm_d, joy_time, send_time)
        packed_data = struct.pack('6f', *float_vars)
        self.sock.sendto(packed_data, (self.server_ip, self.server_port))
        self.get_logger().info("Sent data: {}".format(float_vars))

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
