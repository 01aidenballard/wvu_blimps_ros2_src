
import rclpy
from rclpy.node import Node
import socket
import struct
import threading
import time


class ReceiverNode(Node):

    def __init__(self):
        super().__init__('receiver_node')
        self.receiver_ip = '0.0.0.0'  # Listening on all available interfaces
        self.receiver_port = 12345  # Should match the sender's port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.receiver_ip, self.receiver_port))
        self.declare_parameter('file_name','test_data')
        self.filename = self.get_parameter('file_name').get_parameter_value().string_value
        self.get_logger().info(f"Listening on UDP {self.receiver_ip}:{self.receiver_port}")
        self.listener_thread = threading.Thread(target=self.listen_data)
        self.listener_thread.daemon = True
        self.listener_thread.start()

    def listen_data(self):
        while rclpy.ok():
            data, _ = self.sock.recvfrom(1024)  # 1024 bytes buffer size
            float_vars_received = struct.unpack('6f', data)
            self.get_logger().info('Received data: {}'.format(float_vars_received))
            with open(self.filename, 'a') as file:
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
                line = f"{timestamp}: {float_vars_received}\n"
                file.write(line)

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    receiver_node = ReceiverNode()
    try:
        rclpy.spin(receiver_node)
    except KeyboardInterrupt:
        receiver_node.get_logger().info('Receiver Node Stopped Cleanly')
    except BaseException as e:
        receiver_node.get_logger().error('An exception occurred: {}'.format(e))
    finally:
        receiver_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
