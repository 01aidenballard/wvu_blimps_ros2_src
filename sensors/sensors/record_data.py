import rclpy
from rclpy.node import Node
import math
import time
#from std_msgs.msg import Float32MultiArray #also have to add dependcy package into package.xml
from blimp_interfaces.msg import ImuData
from blimp_interfaces.msg import EscInput


class RecordDataNode(Node): #Creating a Node
    
    def __init__(self): #initiating node

        self.accel = [0,0,0]
        self.gyro = [0,0,0]
        self.euler = [0,0,0]
        self.motor = [0,0,0,0]

        super().__init__('recording_node') #naming node 'recording_data'

        self.subscriber = self.create_subscription(
			ImuData, "imu_data", self.callback_record_imu, 10
		)
        self.subscriber = self.create_subscription(
			EscInput, "ESC_input", self.callback_record_esc, 10
		)

        self.create_timer(0.2, self.callback_record_data) #calls function every 0.2 seconds
        self.start_time = time.time()
    
    def callback_record_imu(self,msg):
        self.accel = msg.imu_lin_accel
        self.gyro = msg.imu_gyro
        self.euler = msg.imu_euler

    def callback_record_esc(self,msg):
        self.motor = msg.esc_pwm

    def callback_record_data(self):
        dt = time.time() - self.start_time
        f = open("data_record_1.txt", 'a')
        f.write("Euler Angles: {},{},{}".format(self.euler[0],self.euler[1],self.euler[2]) + " " + "Gyro: {},{},{}".format(self.gyro[0],self.gyro[1],self.gyro[2]) + " " + 
                "Linear Accel: {},{},{}".format(self.accel[0],self.accel[1],self.accel[2]) + " " + "L Motor: {}".format(self.motor[0]) + " " +
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
