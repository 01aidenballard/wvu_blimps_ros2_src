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

        self.cam_data = self.create_publisher(CameraCoord,"cam_data",3) #Initializing publisher (message type,name,Qsize(some buffer thing:10 messages before it erases last one)S)

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(3,640)  # x-axis
        self.cap.set(4,480)  # y-axis

        self.frame_count = 0
        self.total_x = 0
        self.total_y = 0
        self.minimum_radius = 20

        self.create_timer(0.2, self.callback_read_image) #calls function every 0.2 seconds

        self.get_logger().info("Balloon Detection has Started")

    def callback_read_image(self):
        #tracemalloc.start()
        if not self.cap.isOpened():
            self.get_logger().info("Error: Could not open video source.")
            return

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().info("Error: Could not read frame.")
            return

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        lower_bound_purple = np.array([115, 50, 50])
        upper_bound_purple = np.array([160, 255, 255])

        lower_bound_green = np.array([33, 50, 50])  # Adjusted for light green
        upper_bound_green = np.array([90, 255, 255])  # Adjusted for dark green

        mask_green = cv2.inRange(hsv_frame, lower_bound_green, upper_bound_green)
        mask_purple = cv2.inRange(hsv_frame, lower_bound_purple, upper_bound_purple)

        full_mask = mask_green + mask_purple

        frame = cv2.bitwise_and(frame, frame, mask=full_mask)
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	
	    # apply bilateralFilter to reduce noise. medianBlur is also added for smoothening, reducing noise.
        gray = cv2.bilateralFilter(gray,15,75,75)
        
        gray = cv2.medianBlur(gray,5)

	    # Adaptive Guassian Threshold is to detect sharp edges in the Image. For more information Google it.
        gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,13,3.5)

        kernel = np.ones((5,5),np.uint8)
        
        gray = cv2.erode(gray,kernel,iterations = 1)
        # gray = erosion
	
        gray = cv2.dilate(gray,kernel,iterations = 1)
	    # gray = dilation
	    # detect circles in the image
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 500, param1=75, param2=25, minRadius=10, maxRadius=500)
               
        detected_coordinates = []  # List to store detected circle coordinates
        largest_area = 0
        # ensure at least some circles were found
        if circles is not None:
	        # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            self.frame_count += 1
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                area = r * r * 3.14
                if area > largest_area:
                    detected_coordinates.append((int(x), int(y)))
                    # cv2.circle(output, (x, y), r, (0, 255, 0), 4)


            cv2.imshow('Detected Color', gray)
        if self.frame_count % 3 == 0:
            for idx, (x, y) in enumerate(detected_coordinates):
                total_x = sum(x for x, _ in detected_coordinates)
                total_y = sum(y for _, y in detected_coordinates)
                avg_x = total_x / len(detected_coordinates)
                avg_y = total_y / len(detected_coordinates)
                self.get_logger().info("X: " + str(avg_x) + ", Y: " + str(avg_y))
                msg = CameraCoord()
                msg.position = [int(avg_x),int(avg_y)]
                self.cam_data.publish(msg)
        #snapshot = tracemalloc.take_snapshot()
        #top_stats = snapshot.statistics('lineno')
        #for stat in top_stats[:10]:
        #    print(stat)


def main(args=None):
    rclpy.init(args=args)
    node = CamNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': #this allows us to run script from terminal directly
    main()
