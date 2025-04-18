# import rclpy								#ros2 library for python
# from rclpy.node import Node					#same as above
# from sensor_msgs.msg import Joy				#importing the Joy interface that was downloaded in the joy package
# import time									# importing time
# import pigpio								# gpio library
# import os
# os.system("sudo pigpiod")
# #import RPi.GPIO as GPIO
# # time.sleep(1)

# LED_pin = 26
# #GPIO.setmode(GPIO.BOARD)
# #GPIO.setup(LED_pin, GPIO.OUT)

# class LED_Modulation(Node):
#     def __init__(self):
# 		# defining the LED Pin
#         self.LED_state = True
#         self.LED_pin1 = 26
#         self.LED_pin2 = 19
#         self.pi = pigpio.pi()

#         #Initializing the node and nameing it "led" 
#         super().__init__("led")

#         #Subscribing to the /joy topic with is the controller read information
#         self.subscriber = self.create_subscription(Joy, "joy", self.callback_LED_input, 10)
#         time.sleep(1)

#     def callback_LED_input(self,msg):
#         if msg.buttons[6] == 1:
#             self.LED_state = not self.LED_state
#             time.sleep(1)

#         if self.LED_state is False:
#             #GPIO.output(LED_pin, GPIO.HIGH)
#             self.pi.write(self.LED_pin1, 1)
#             self.pi.write(self.LED_pin2, 1)
#             self.get_logger().info("LED should be off")
#         else:
#             self.pi.write(self.LED_pin1, 0)
#             self.pi.write(self.LED_pin2, 0)
# def main(args=None):
#     rclpy.init(args=args)
#     node = LED_Modulation()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()

import rclpy                                # ros2 library for python
from rclpy.node import Node                 # same as above
from sensor_msgs.msg import Joy             # importing the Joy interface
import time                                 # importing time
import pigpio                               # gpio library
import os

os.system("sudo pigpiod")
time.sleep(1)  # Give pigpiod time to start

class LED_Modulation(Node):
    def __init__(self):
        super().__init__("led")

        # Define GPIO pins for each LED
        self.LED_pin1 = 18  # LED 1
        self.LED_pin2 = 23  # LED 2
        self.LED_pin3 = 24  # LED 3

        # Initial LED state: ON
        self.LED_state = True

        # Init pigpio
        self.pi = pigpio.pi()

        # Ensure all pins are set to output low initially
        self.pi.write(self.LED_pin1, 0)
        self.pi.write(self.LED_pin2, 0)
        self.pi.write(self.LED_pin3, 0)

        # Subscribe to /joy topic for controller input
        self.subscriber = self.create_subscription(Joy, "joy", self.callback_LED_input, 10)

        self.get_logger().info("LED node started successfully")


    def callback_LED_input(self, msg):
        self.get_logger().info(f"Button state: {msg.buttons[6]} | Current LED state: {self.LED_state}")
        # Toggle LED state on button 6 press
        if msg.buttons[6] == 1:
            self.LED_state = not self.LED_state
            time.sleep(1)  # Debounce

        if not self.LED_state:
            self.pi.write(self.LED_pin1, 1)
            self.pi.write(self.LED_pin2, 1)
            self.pi.write(self.LED_pin3, 1)
            self.get_logger().info("LEDs should be OFF (HIGH)")
        else:
            self.pi.write(self.LED_pin1, 0)
            self.pi.write(self.LED_pin2, 0)
            self.pi.write(self.LED_pin3, 0)
            self.get_logger().info("LEDs ON (LOW)")

def main(args=None):
    rclpy.init(args=args)
    node = LED_Modulation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

