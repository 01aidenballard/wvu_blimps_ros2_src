import rclpy
from rclpy.node import Node
import time
import board
import adafruit_bmp3xx # libraries for barometer, adafruit_bmp3xx
from blimp_interfaces.msg import BaroData

class BarometerNode(Node): #Creating a Node

        def __init__(self):
                super().__init__('barometer_node')
                self.barometer_data = self.create_publisher(BaroData,"barometer_data",10)
                self.i2c = board.I2C()
                self.sensor = adafruit_bmp3xx.BMP3XX_I2C(self.i2c)
                #adafruit_bmp3xx.BMP3XX.filter_coefficient(128)
                self.declare_parameter('sea_level_pressure', 999.3233801073472)
                self.sea_level_pressure = self.get_parameter('sea_level_pressure').value
                self.sensor.sea_level_pressure = self.sea_level_pressure
                self.height_init =0.0
                self.c = 0

        def publish_barometer_data(self):
                while True:
                        msg = BaroData()

                        if self.c == 0:
                                self.height_init = self.sensor.altitude
                                self.c = self.c + 1
                        
                        msg.height = float(self.sensor.altitude) - self.height_init
                        #self.get_logger().info(str(msg)) # Prints data to command line
                        self.barometer_data.publish(msg)

                        time.sleep(1)
def main(args=None):
        rclpy.init(args=args)
        node = BarometerNode()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__': #this allows us to run script from terminal directly
        main()
