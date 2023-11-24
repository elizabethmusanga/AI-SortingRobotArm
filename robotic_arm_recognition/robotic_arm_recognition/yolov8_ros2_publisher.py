#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from robotic_arm_msgs.msg import ObjectInference
from robotic_arm_msgs.msg import Yolov8Inference

bridge = CvBridge()

# Constants for the picking height of the robotic arm
PICKING_HEIGHT = 0.05

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO("/home/newton/ROS2/ai_based_sorting_robotic_arm/src/AI-SortingRobotArm/Computer Vision Models/5best.pt")

        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            'rgb_cam/image_raw',
            self.camera_callback,
            10)
        self.subscription 

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/object_inference", 1)

    def camera_callback(self, data):

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.object_inference = ObjectInference()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                x_center = (b[0] + b[2]) / 2
                y_center = (b[1] + b[3]) / 2
                self.object_inference.class_name = self.model.names[int(c)]
                self.object_inference.x_coord = x_center
                self.object_inference.y_coord = y_center
                self.object_inference.z_coord = PICKING_HEIGHT
                self.yolov8_inference.yolov8_inference.append(self.object_inference)

            #camera_subscriber.get_logger().info(f"{self.yolov8_inference}")

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()