#!/usr/bin/env python3
import rclpy
import serial
import json
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String


# Data format of the joint angles to be send serialized to arduino
joint_angles =     '''{"joint_1":10,"joint_2":20,"joint_3":30,"joint_4":40}'''



class SerialTransmitter(Node):
    def __init__(self):
        super().__init__("serial_transmitter")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        
        # Variables to store joint angles
        self.joint_1 = 0.0
        self.joint_2 = 0.0
        self.joint_3 = 0.0
        self.joint_4 = 0.0

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value
        self.publish_topic = "serialised_joint_angles"
        self.subscription_topic = "/robot_joints_joint_trajectory_controller/joint_trajectory"
        
        # Serial object
        self.arduino_serial = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)
        
        # Subscriber to arduino_serial
        self.subscriber = self.create_subscription(JointTrajectory, self.subscription_topic, self.msg_callback, 10)
      
        # Publisher to /robot_joints_joint_trajectory_controller/joint_trajectory topic
        self.publisher = self.create_publisher(String, self.publish_topic, 10)
        # Timer for publishing
        self.publish_timer = self.create_timer(1.0, self.publish_joint_angles)
        

    # Publisher callback function
    def publish_joint_angles(self):
        joint_angles = String()
        joint_angles = f'''  "joint_1":  {self.joint_1},
                            "joint_2":  {self.joint_2},
                            "joint_3":  {self.joint_3},
                            "joint_4":  {self.joint_4}
                        '''
        self.publisher.publish(joint_angles)
     
     # Subscriotion callback   
    def get_joint_angles(self, msg: JointTrajectory):
        self.get_logger().info(f"Joints angles = {msg.points}")
        self.joint_1 = msg.points[0]
        self.joint_2 = msg.points[1]
        self.joint_3 = msg.points[2]
        self.joint_4 = msg.points[3]
        
    def msg_callback(self, msg):
        self.get_logger().info("New message received, publishing on serial: %s" % self.arduino_serial.name)
        self.arduino_serial.write(msg.data.encode("utf-8"))


def main():
    rclpy.init()

    serial_transmitter = SerialTransmitter()
    rclpy.spin(serial_transmitter)
    
    serial_transmitter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()