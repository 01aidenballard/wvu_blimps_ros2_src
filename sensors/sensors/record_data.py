import rclpy
from rclpy.node import Node
import math
import time
#from std_msgs.msg import Float32MultiArray #also have to add dependcy package into package.xml
from blimp_interfaces.msg import EscInput
from blimp_interfaces.msg import CameraCoord
from sensor_msgs.msg import Joy

class RecordDataNode(Node): #Creating a Node
    
    def __init__(self): #initiating node

        
        self.Manual_mode  = True
        super().__init__('recording_node') #naming node 'recording_data'

        self.subscriber = self.create_subscription(
			CameraCoord, "cam_data", self.callback_record_cam, 10
		)
        self.subscriber = self.create_subscription(
			EscInput, "ESC_input", self.callback_record_esc, 10
		)

        self.subscriber = self.create_subscription(
			Joy, "joy", self.callback_record_data, 10
		)

        self.create_timer(0.2, self.callback_record_data) #calls function every 0.2 seconds
        self.start_time = time.time()
    
    def callback_record_cam(self,msg):
        self.cam = msg.position

    def callback_record_esc(self,msg):
        self.motor = msg.esc_pwm

    def callback_record_data(self,msg):

        if msg.buttons[0]  == 1:
            self.Manual_mode = not self.Manual_mode
            time.sleep(2)
            self.get_logger().info("Manual Mode is " + str(self.Manual_mode))
            self.start_time = time.time()

        if self.manual_mode is False:
            dt = time.time() - self.start_time
            f = open("PI_Data_1.txt", 'a')
            f.write("Position: {},{}".format(self.position[0],self.position[1])  + " L Motor: {}".format(self.motor[0]) + " " +
                    "R Motor: {}".format(self.motor[1]) + " " + "U Motor: {}".format(self.motor[2]) + " " + 
                    "D Motor: {}".format(self.motor[3]) + " " + "time: {}".format(dt) + "\n")
            f.close()

        
def main(args=None):
    rclpy.init(args=args)
    node = RecordDataNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': #this allows us to run script from terminal directly
    main()
