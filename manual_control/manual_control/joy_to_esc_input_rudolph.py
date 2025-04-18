# # This is for Single Directional ESCs
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Joy
# from blimp_interfaces.msg import EscInput

# class FixAxesNode(Node):
#     def __init__(self):
#         # Defining the pin numbers for each motor
#         self.ESC_pin1 = 5  # Left Motor Pin
#         self.ESC_pin2 = 6  # Right Motor Pin
#         self.ESC_pin3 = 13 # Vertical Motor Pin
#         self.ESC_pin4 = 26 # Adiitonal Motor Pin

#         # Initializing the node
#         super().__init__("joy_to_esc")

#         # Declaring parameters for motor scaling
#         self.declare_parameter('Klm', 1.0)
#         self.declare_parameter('Krm', 1.0)
#         self.Klm = self.get_parameter('Klm').value
#         self.Krm = self.get_parameter('Krm').value
        
#         # Subscribing to the /joy topic for controller input
#         self.subscriber = self.create_subscription(
#             Joy, "joy", self.callback_manual_esc_input, 10
#         )
        
#         # Setting up the publisher for ESC input
#         self.publisher = self.create_publisher(EscInput, "ESC_Manual_input", 10)
#         self.get_logger().info("Data is being sent to the ESC node")

#     def callback_manual_esc_input(self, msg):
#         # Get joystick inputs for the left and right joystick
#         joystick_left_horizontal = msg.axes[0]  # Left joystick horizontal (left/right movement)
#         joystick_left_vertical = msg.axes[1]  # Left joystick vertical (up/down movement)
        
#         joystick_right_vertical = msg.axes[2] # Right joystick vertical (up/down movement)
#         joystick_right_horizontal = msg.axes[4] # Right joystick horizontal (left/right movement)
        
#         # Get trigger values (values range from 0 to 1)
#         left_trigger = msg.axes[5]  # Left trigger (0 to -1, where Unpressed = 0)
#         right_trigger = msg.axes[3] # Right trigger (1 to -1, where Unpressed = 1)

#         # For the left joystick, control both left and right motors (LM and RM)
#         if joystick_left_vertical > 0:  # Joystick moved up
#             pwm_value = 1050 + (joystick_left_vertical * (1950 - 1050))  # Scale from 1050 to 1950
#         else:
#             pwm_value = 1050  # If joystick moved down or centered, stay at 1050

#         # For the right joystick, control the vertical motor (DM)
#         if joystick_right_vertical > 0:  # Joystick moved up
#             DM_pwm = 1050 + (joystick_right_vertical * (1950 - 1050))  # Scale from 1050 to 1950
#         else:
#             DM_pwm = 1050  # If joystick moved down or centered, stay at 1050

#         # Initialize the motor PWM values
#         # The third lateral motor (ESC_pin4) will follow the same logic as LM and RM
#         # Treating it like an extra side thruster in the pizza slice setup
#         EX_pwm = pwm_value  # Initialize with same logic as LM and RM
#         LM_pwm = pwm_value
#         RM_pwm = pwm_value

#         # Apply trigger overrides if needed (just like LM and RM)
#         if left_trigger < 0:
#             EX_pwm = 1050  # Disable third motor if left trigger is overriding
#         if right_trigger < 0:
#             EX_pwm = 1050  # Disable third motor if right trigger is overriding

#         # If the left trigger is pressed, control only the right motor
#         if left_trigger < 0:  # Trigger is pressed
#             RM_pwm = 1050 + ((-left_trigger) * (1950 - 1050))  # Scale from 1050 to 1950
#             LM_pwm = 1050  # Keep the left motor (LM) at 1050

#         # If the right trigger is pressed, control only the left motor
#         if right_trigger < 0:  # Trigger is pressed
#             LM_pwm = 1050 + ((-right_trigger) * (1950 - 1050))  # Scale from 1050 to 1950
#             RM_pwm = 1050  # Keep the right motor (RM) at 1050

#         # Ensure the PWM values are floats
#         LM_pwm = float(LM_pwm)
#         RM_pwm = float(RM_pwm)
#         DM_pwm = float(DM_pwm)
#         EX_pwm = float(EX_pwm)
#         EX_pwm = float(EX_pwm)


#         # Create the message to be sent to the ESC
#         msg2 = EscInput()
#         msg2.esc_pins = [self.ESC_pin1, self.ESC_pin2, self.ESC_pin3, self.ESC_pin4]
#         msg2.pwm_l = LM_pwm
#         msg2.pwm_r = RM_pwm
#         msg2.pwm_d = DM_pwm
#         msg2.pwm_extra = EX_pwm  # Add new field if needed in EscInput.msg

#         # Publish the message
#         self.publisher.publish(msg2)

# def main(args=None):
#     rclpy.init(args=args)
#     node = FixAxesNode()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()

# This is for Single Directional ESCs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from blimp_interfaces.msg import EscInputRudolph
import math


class FixAxesNode(Node):
    def __init__(self):
        # Defining the pin numbers for each motor
        self.SM_pin_1 = 5   # Side Motor 1
        self.SM_pin_2 = 6   # Side Motor 2
        self.SM_pin_3 = 13  # Side Motor 3
        self.VM_pin    = 26  # Vertical Motor

        # Initializing the node
        super().__init__("joy_to_esc_rudolph")

        # Declaring parameters for motor scaling
        self.declare_parameter('Klm', 1.0)
        self.declare_parameter('Krm', 1.0)
        self.Klm = self.get_parameter('Klm').value
        self.Krm = self.get_parameter('Krm').value

        # Subscribing to the /joy topic for controller input
        self.subscriber = self.create_subscription(
            Joy, "joy", self.callback_manual_esc_input, 10
        )

        # Setting up the publisher for ESC input
        self.publisher = self.create_publisher(EscInputRudolph, "ESC_Manual_input", 10)
        self.get_logger().info("Data is being sent to the ESC node")

    def callback_manual_esc_input(self, msg):
        # Get left joystick inputs
        x = msg.axes[0]  # Left stick horizontal
        y = msg.axes[1]  # Left stick vertical

        # Get right joystick vertical input (for vertical motor)
        joystick_right_vertical = msg.axes[2]

        # Compute joystick angle in degrees
        angle = (math.degrees(math.atan2(y, x)) + 360) % 360
        magnitude = (x**2 + y**2)**0.5

        # Scale magnitude to PWM (1050 to 1950)
        pwm = 1050 + min(magnitude, 1.0) * (1950 - 1050)

        # Default: all side motors off
        SM_1_PWM = 1050.0
        SM_2_PWM = 1050.0
        SM_3_PWM = 1050.0

        # Map angle to slice
        if 0 <= angle < 120:
            SM_1_PWM = pwm
        elif 120 <= angle < 240:
            SM_2_PWM = pwm
        else:  # 240 to 360
            SM_3_PWM = pwm

        # Vertical motor
        if joystick_right_vertical > 0:
            VM_PWM = 1050 + joystick_right_vertical * (1950 - 1050)
        else:
            VM_PWM = 1050.0

        # Build and publish message
        msg2 = EscInputRudolph()
        msg2.esc_pins = [self.SM_pin_1, self.SM_pin_2, self.SM_pin_3, self.VM_pin]
        msg2.pwm_sm1 = SM_1_PWM
        msg2.pwm_sm2 = SM_2_PWM
        msg2.pwm_sm3 = SM_3_PWM
        msg2.pwm_vm = VM_PWM



        # self.get_logger().info(
        #     f"\n[Joystick]\n"
        #     f"  Left Stick  -> x: {x:.2f}, y: {y:.2f}\n"
        #     f"  Angle: {angle:.1f}°, Magnitude: {magnitude:.2f}, PWM: {pwm:.1f}\n"
        #     f"[Motor PWM Output]\n"
        #     f"  SM1: {SM_1_PWM:.1f}, SM2: {SM_2_PWM:.1f}, SM3: {SM_3_PWM:.1f}, VM: {VM_PWM:.1f}"
        # )

        self.publisher.publish(msg2)


def main(args=None):
    rclpy.init(args=args)
    node = FixAxesNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
