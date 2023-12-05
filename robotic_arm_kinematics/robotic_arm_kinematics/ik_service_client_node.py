#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from numpy import pi
from robotic_arm_msgs.srv import IKSolver
from robotic_arm_msgs.msg import Yolov8Inference
import keyboard

class IKClientNode(Node):
    def __init__(self):
        super().__init__('ik_client_node')
        # 
        self.send_command: bool = False
 
        # List to store previous detected objects and their positions in the scene
        self.previous_positions = [" ", " ", " ", " "]


        # Z Position of the gripper when placing an object
        self.placing_coord_z = 200

        # Robot's home position
        self.home_pos = [13, -222, 292]

        # List to store positions of the cylinders, cubes, and hexagons containers respectively
        self.cylinders_container_pos = [-240, -70, 55]
        self.cubes_container_pos = [-210, -295, 55]
        self.hexagons_container_pos = [-130, -70, 55]

        # Gripper States and their corresponding angles
        self.GRIPPER_OPEN = pi/3
        self.GRIPPER_CLOSE = 0

        # List to store the variable of where the object picked and is to be placed
        self.start_position = self.home_pos
        self.end_position = self.home_pos

        # Subscriber to the Yolov8 inference topic
        self.pos_subscriber = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.object_inference_callback,
            10)
        

    # Service client setup
    def service_client_setup(self, x, y, z, gripper_state):
        self.client_ = self.create_client(IKSolver, 'ik_server')

        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service not available, waiting again...')

        self.request = IKSolver.Request()
        self.request.x_coord = x
        self.request.y_coord = y
        self.request.z_coord = z
        self.request.gripper_state = gripper_state

        self.future_ = self.client_.call_async(self.request)
        self.future_ = self.future_.add_done_callback(self.response_callback)


    # Callback function for the subscriber
    def object_inference_callback(self, msg):
        class_names = msg.class_names
        object_positions = msg.detected_obj_positions

        # Construct a list of lists from the class names and object positions
        objects_and_positions = self.construct_2d_list(class_names, object_positions)


        self.get_logger().info(f"Received class names: {class_names}")
        self.get_logger().info(f"Received object positions: {objects_and_positions}")
        self.get_logger().info(f"Previous object positions: {self.previous_positions}")

        # If the object positions are the same as the previous positions, do nothing 
        if not self.compare_lists(objects_and_positions, self.previous_positions):
            object_to_pick = self.first_object_to_pick_determiner(objects_and_positions)
            self.get_logger().info("Object positions changed")
            class_name = object_to_pick[0]
            x = int(object_to_pick[1])
            y = int(object_to_pick[2])
            z = int(object_to_pick[3])

            self.start_position = [x, y, z]
            if class_name == "Cylinder":
                self.end_position = self.cylinders_container_pos
            elif class_name == "Cube":
                self.end_position = self.cubes_container_pos
            elif class_name == "Hexagon":
                self.end_position = self.hexagons_container_pos
            else:
                self.end_position = self.home_pos

            self.previous_positions = objects_and_positions

            self.pick_and_place_object()
        else:
            self.get_logger().info("No new objects detected")

            # if keyboard.read_key() == 'k':
            #     self.send_command = True

                
            # if self.send_command:
            #     self.pick_and_place_object()

            # self.get_logger().info(f"Received object positions: {object_positions}")

    # Callback function for the service client
    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received joint angles: {response.joint_angles}")
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")


    # Function to open and close the gripper
    def open_close_gripper(self, gripper_state: bool):
        if gripper_state == True:
            print("Opening gripper")
            return self.GRIPPER_OPEN
        else:
            print("Closing gripper")
            return self.GRIPPER_CLOSE
        
        # Function to decide what Object to pick first
    def first_object_to_pick_determiner(self, objects_list):
        x_closest = objects_list[0][1]
        object_to_pick = objects_list[0]
        # Pick object closest to the robot's base position wrt x axis
        for object in range(len(objects_list)):
            if objects_list[object][1] < x_closest:
                x_closest = objects_list[object][1]
                object_to_pick = objects_list[object]

        # # Remove the object from the list
        # objects_list.remove(object_to_pick)
        return object_to_pick
            
    # Define the pick and place logic
    def pick_and_place_object(self):

        # Open gripper
        self.service_client_setup(self.home_pos[0], self.home_pos[1], self.home_pos[2], True)
        # Move to the object's position
        self.service_client_setup(self.start_position[0], self.start_position[1], self.start_position[2], True)
        # Close the gripper
        self.service_client_setup(self.start_position[0], self.start_position[1], self.start_position[2], False)
        # Move to the placing position
        self.service_client_setup(self.end_position[0], self.end_position[1], self.end_position[2], False)
        # Open the gripper
        self.service_client_setup(self.end_position[0], self.end_position[1], self.end_position[2], True)

        # Return to home position
        self.service_client_setup(self.home_pos[0], self.home_pos[1], self.home_pos[2], False)

        # self.send_command = False

    # Function to compare two lists
    def compare_lists(self, list1, list2):
        list1 = sorted(list1)
        list2 = sorted(list2)
        if list1 == list2:
            return True
        else:
            return False

    # Function to construct a list of lists from 2 lists
    def construct_2d_list(self, list1, list2):
        list_of_lists = []

        for i in range(len(list1)):
            list_of_lists.append([list1[i], list2[i * 3], list2[i * 3 + 1], list2[i * 3 + 2]])

        return list_of_lists


def main():
    rclpy.init(args=None)
    ik_client_node = IKClientNode()
    rclpy.spin(ik_client_node)
    ik_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()