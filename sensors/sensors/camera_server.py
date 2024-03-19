import rclpy
from rclpy.node import Node
import time
import cv2
import numpy as np
import math
from blimp_interfaces.msg import CameraCoord


class CamNode(Node): #Creating a Node

    def __init__(self): #initiating node

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(3,480 )  # x-axis
        self.cap.set(4, 480)  # y-axis

        self.frame_count = 0
        self.total_x = 0
        self.total_y = 0
        self.minimum_radius = 20

        if not self.cap.isOpened():
                self.get_logger().info("Error: Could not open video source.")
                return

        super().__init__("camera_server")
        self.server_ = self.create_service(CameraCoord, "camera_data", self.callback_read_image)
        self.get_logger().info("Balloon Detection has Started")

    def callback_read_image(self, request, response):
        if request == 1: #1 means start takeing cam data

            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().info("Error: Could not read frame.")
                return

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
                    detected_coordinates.append((center[0], center[1]))

            self.frame_count += 1
            if self.frame_count % 10 == 0:
                for idx, (x, y) in enumerate(detected_coordinates):
                    total_x = sum(x for x, _ in detected_coordinates)
                    total_y = sum(y for _, y in detected_coordinates)
                    avg_x = total_x / len(detected_coordinates)
                    avg_y = total_y / len(detected_coordinates)
                    
                    response.position = [int(avg_x),int(avg_y)]
            return response



def main(args=None):
    rclpy.init(args=args)
    node = CamNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': #this allows us to run script from terminal directly
    main()
