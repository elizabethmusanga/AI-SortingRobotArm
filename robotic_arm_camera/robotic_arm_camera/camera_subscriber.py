#!usr/bin/env python3
#-*- coding:utf-8 _*-

"""
@Author: Guyueju (www.guyuehome.com)
@Description: ROS2 Topic Example - Subscribing to Image Topic
"""

import rclpy                            # ROS2 Python interface library
from rclpy.node import Node             # ROS2 Node class
from sensor_msgs.msg import Image       # Image message type
from cv_bridge import CvBridge          # ROS and OpenCV image conversion class
import cv2                              # OpenCV image processing library
from ultralytics import YOLO            # YOLOv5 object detection library
import numpy as np                      # Python numerical computing library
import cv2.aruco as aruco               # ArUco marker library

from robotic_arm_msgs.msg import ObjectInference    # Custom message type
from robotic_arm_msgs.msg import Yolov8Inference    # Custom message type


# Define the picking height of the robotic arm
PICKING_HEIGHT = 50

"""
Create a subscriber node 
"""

class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                  # Initialize the ROS2 node parent class
        
        self.aruco_dict_type = aruco.DICT_4X4_50                # Define the ArUco marker dictionary type
        self.aruco_marker_size_mm = 34                          # Define the ArUco marker size in mm
        
        self.camera_matrix = np.array([[650, 0, 320], [0, 650, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)

        self.model = YOLO("/home/newton/ROS2/ai_based_sorting_robotic_arm/src/AI-SortingRobotArm/Computer Vision Models/5best.pt") # Load the YOLOv5 model  
        
        self.yolov8_inference = Yolov8Inference()               # Create a custom message object
        self.object_inference = ObjectInference()               # Create a custom message object


        self.sub = self.create_subscription(
            Image, 'image_raw', self.camera_call_back, 10)      # Create a subscriber object (message type, topic name, subscriber callback function, queue length)
        
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1) 
        self.img_pub = self.create_publisher(Image, "/object_inference", 1)             
        self.cv_bridge = CvBridge()                             # Create an image conversion object for converting between OpenCV images and ROS image messages

    def detect_and_position_object(self, image):
        
        reference_center, pixel_to_mm_ratio, aruco_centers = self.detect_aruco_and_get_reference_center(image)

        results = self.model(image)

        if reference_center is not None and pixel_to_mm_ratio is not None:
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    self.object_inference = ObjectInference()
                    b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                    c = box.cls
                    x_center = ((b[0] + b[2]) / 2)
                    y_center = ((b[1] + b[3]) / 2)
                    x_center_mm = int((x_center - reference_center[0]) * pixel_to_mm_ratio)
                    y_center_mm = int((y_center - reference_center[1]) * pixel_to_mm_ratio)

                    

                    self.object_inference.class_name = self.model.names[int(c)]
                    self.object_inference.x_coord = x_center_mm
                    self.object_inference.y_coord = y_center_mm
                    self.object_inference.z_coord = PICKING_HEIGHT

                    if self.object_inference.class_name == "Cylinder" or self.object_inference.class_name == "Cube" or self.object_inference.class_name == "Hexagon":
                        self.yolov8_inference.class_names.append(self.object_inference.class_name)
                        self.yolov8_inference.detected_obj_positions.append(str(self.object_inference.x_coord))
                        self.yolov8_inference.detected_obj_positions.append(str(self.object_inference.y_coord))
                        self.yolov8_inference.detected_obj_positions.append(str(self.object_inference.z_coord))

            annotated_frame = results[0].plot()
            img_msg = self.cv_bridge.cv2_to_imgmsg(annotated_frame)  

            self.img_pub.publish(img_msg)
            self.yolov8_pub.publish(self.yolov8_inference)
            self.yolov8_inference.class_names.clear()
            self.yolov8_inference.detected_obj_positions.clear()
            self.resize_frame(annotated_frame, 650, 350)
            cv2.imshow("YOLOv5", annotated_frame)                    # Display the image
            cv2.waitKey(1)                                           # Wait for a key press

    def camera_call_back(self, data):
        self.get_logger().info('Receiving video frame')         # Output log information indicating that the callback function has been entered
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')      # Convert the ROS image message to an OpenCV image
        self.detect_and_position_object(image)                  # Object detection and positioning


    
    def detect_aruco_and_get_reference_center(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dictionary = aruco.getPredefinedDictionary(self.aruco_dict_type)
        parameters = aruco.DetectorParameters()
        # detector = aruco.ArucoDetector(dictionary, parameters)
        markerCorners, markerIds, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)

        reference_center = None
        pixel_to_mm_ratio = None
        centers = {}

        if markerCorners and markerIds is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(markerCorners, self.aruco_marker_size_mm,
                                                             self.camera_matrix, self.dist_coeffs)
            for idx, corners in enumerate(markerCorners):
                int_corners = np.int0(corners)
                center = np.mean(int_corners[0], axis=0)
                centers[int(markerIds[idx])] = center
                if markerIds[idx] == 1:
                    reference_center = center
                    aruco_side_pixels = np.linalg.norm(int_corners[0][0] - int_corners[0][1])
                    pixel_to_mm_ratio = self.aruco_marker_size_mm / aruco_side_pixels

                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[idx], tvecs[idx],
                                  self.aruco_marker_size_mm / 2)

        aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
        return reference_center, pixel_to_mm_ratio, centers
    

    def resize_frame(self, frame, target_width=1366, target_height=768):
        height, width = frame.shape[:2]
        if width > target_width or height > target_height:
            scaling_factor = min(target_width / width, target_height / height)
            frame = cv2.resize(frame, None, fx=scaling_factor, fy=scaling_factor, interpolation=cv2.INTER_AREA)
        return frame
    

 


def main(args=None):                                        # ROS2 node main entry point main function
    rclpy.init(args=args)                                   # Initialize the ROS2 Python interface
    node = ImageSubscriber("topic_webcam_sub")              # Create a ROS2 node object and initialize it
    rclpy.spin(node)                                        # Loop and wait for ROS2 to exit
    node.destroy_node()                                     # Destroy the node object
    rclpy.shutdown()                                        # Close the ROS2 Python interface

