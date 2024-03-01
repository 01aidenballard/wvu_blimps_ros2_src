
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
        self.create_timer(0.2, self.publish_cam_data) #calls function every 0.2 seconds
        self.minimum_radius = 20

        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 640)  # x-axis
        self.cap.set(4, 480)  # y-axis

        if not self.cap.isOpened():
            print("Error: Could not open video source.")
            return
        
        self.frame_count = 0
        self.total_x = 0
        self.total_y = 0

    def publish_cam_data(self):
        ret, frame = self.cap.read()
        
        if not ret:
            print("Error: Could not read frame.")

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_bound_1 = np.array([56, 41, 155])
        upper_bound_1 = np.array([76, 81, 255])
            
        lower_bound_2 = np.array([116, 16, 154]) 
        upper_bound_2 = np.array([136, 116, 254])

        mask_1 = cv2.inRange(hsv_frame, lower_bound_1, upper_bound_1)
        mask_2 = cv2.inRange(hsv_frame, lower_bound_2, upper_bound_2)

        contours_1, _ = cv2.findContours(mask_1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_2, _ = cv2.findContours(mask_2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        all_contours = contours_1 + contours_2

        largest_contour = None
        largest_contour_area = 0

        for contour in all_contours:
            contour_area = cv2.contourArea(contour)
            if contour_area > largest_contour_area:
                largest_contour = contour
                largest_contour_area = contour_area

        detected_coordinates = []  # List to store detected circle coordinates

        if largest_contour is not None:
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            center = (int(x), int(y))
            radius = int(radius)
            if radius >= self.minimum_radius:
                x_direction = "Left" if center[0] > 320 else "Right" if center[0] < 320 else "Center"
                y_direction = "Up" if center[1] < 240 else "Down" if center[1] > 240 else "Center"
                detected_coordinates.append((center[0], center[1], x_direction, y_direction))
                cv2.circle(frame, center, radius, (0, 255, 0), 2)  # Draw a green circle on the frame
            
        cv2.imshow('Detected Color', frame)

        self.frame_count += 1     

        if self.frame_count % 10 == 0:
                
            for idx, (x, y, x_direction, y_direction) in enumerate(detected_coordinates):
                self.total_x = sum(x for x, _, _, _ in detected_coordinates)
                self.total_y = sum(y for _, y, _, _ in detected_coordinates)
                avg_x = self.total_x / len(detected_coordinates)
                avg_y = self.total_y / len(detected_coordinates)
                self.get_logger().info("X: " + str(avg_x) + ", Y: " + str(avg_y))
                msg = CameraCoord()
                msg.x_pos = avg_x
                msg.y_pos = avg_y
                self.cam_data.publish(msg)



        
def main(args=None):
    rclpy.init(args=args)
    node = CamNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': #this allows us to run script from terminal directly
    main()

 
