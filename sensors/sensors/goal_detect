
import rclpy
from rclpy.node import Node
import time
import cv2
import numpy as np
import math
from blimp_interfaces.msg import CameraCoord


class CamNode(Node): #Creating a Node
    
    def __init__(self): #initiating node

        super().__init__('cam_node')

        self.cam_data = self.create_publisher(CameraCoord,"cam_data",10) #Initializing publisher (message type,name,Qsize(some buffer thing:10 messages before it erases last one)S)

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(3, 640)  # x-axis
        self.cap.set(4, 480)  # y-axis

        if not self.cap.isOpened():
            print("Error: Could not open video source.")
            return
        
        self.frame_count = 0
        self.total_x = 0
        self.total_y = 0
        self.minimum_radius = 20
        
        self.total_lines = 0
                    
        self.total_center_x = 0
        self.total_center_y = 0
        self.total_centers = 0

        self.create_timer(0.2, self.callback_check_image()) #calls function every 0.2 seconds

        self.get_logger().info("Balloon Detection has Started")

    def callback_check_image(self):
        ret, frame = self.cap.read()
        
        if not ret:
            print("Error: Could not read frame.")
        else:
            self.callback_read_image(frame)

    def callback_read_image(self,frame):

        
        self.get_logger().info("pass1")
        if not self.cap.isOpened():
            print("Error: Could not open video source.")
            return
    
        while True:
            ret, frame = self.cap.read()


            if not ret:
                print("Error: Could not read frame.")
                break

               
        
            img = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
            lower_bound = np.array([20, 80, 80])
            upper_bound = np.array([45, 255, 255])

            mask_1 = cv2.inRange(hsv_frame, lower_bound, upper_bound)
            low_threshold = 250
            high_threshold = 300
            edges = cv2.Canny(mask_1, low_threshold, high_threshold)

            rho = 1  # distance resolution in pixels of the Hough grid
            theta = np.pi / 180  # angular resolution in radians of the Hough grid
            threshold = 75  # minimum number of votes (intersections in Hough grid cell)
            min_line_length = 50  # minimum number of pixels making up a line
            max_line_gap = 15  # maximum gap in pixels between connectable line segments

            # Run Hough on edge detected image
            # Output "lines" is an array containing endpoints of detected line segments
            linesP = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                                min_line_length, max_line_gap)
            if linesP is not None:
                for line in linesP:
                    for x1, y1, x2, y2 in line:
                        mid_x = (x1 + x2) // 2
                        mid_y = (y1 + y2) // 2

                        self.total_x += mid_x
                        self.total_y += mid_y
                            

                        total_lines += 1
                        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)      #change frame to edges if we are using both line transformers

                if total_lines > 0:
                    # calculate the average center
                    center_x = self.total_x // total_lines
                    center_y = self.total_y // total_lines
                        
                        
                    total_center_x += center_x
                    total_center_y += center_y
                            
                    self.total_centers += 1
                   

                    if total_lines % 10 == 0:
                        avg_center_x = total_center_x // self.total_centers
                        avg_center_y = total_center_y // self.total_centers
                        cv2.circle(frame, (avg_center_x, avg_center_y), 5, (0, 0, 255), -1)
                        self.get_logger().info("Goal X: " + str(avg_center_x) + ", Y: " + str(avg_center_y))
                        msg = CameraCoord()
                        msg.x_pos = avg_center_x
                        msg.y_pos = avg_center_y
                        self.cam_data.publish(msg)



        
def main(args=None):
    rclpy.init(args=args)
    node = CamNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': #this allows us to run script from terminal directly
    main()

 
