#!/usr/bin/env python3

import os
import numpy as np
import rclpy
import ikpy.chain
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor


from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from ament_index_python.packages import get_package_share_directory

from robotic_arm_msgs.srv import IKSolver

# URDF file of the robot
urdf_file = os.path.join(get_package_share_directory("robotic_arm_description"), "urdf", "robotic_arm_urdf.urdf")  

class IKServerNode(Node):
    def __init__(self):
        super().__init__('ik_server_node')

        # Create a service server for finding the IK solution
        self.service_ = self.create_service(IKSolver, "ik_server", self.ik_solver_callback)
        self.get_logger().info("Starting IK Server Node")

        # # Topic to publish to
        # publish_topic = "/robotic_arm_joint_trajectory_controller/joint_trajectory"
        # # Creating the trajectory publisher
        # self.trajectory_publisher = self.create_publisher(JointTrajectory, publish_topic, 10)
        # self.timer_period = 1.0

        
        # Joints to be controlled
        self.joints = [ "base_waist_joint",
                        "waist_link1_joint",
                        "link1_link2_joint",
                        "link2_gripper_base_joint",
                        "right_gripper_joint"
                        ]
        





####################################################################################################################
        self.declare_parameter("controller_name", "robotic_arm_joint_trajectory_controller")
        self.declare_parameter("wait_sec_between_publish", 10)

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        self.wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value


        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.get_logger().info(
            f'Publishing  goals on topic "{publish_topic}"\
              every {self.wait_sec_between_publish} s'
        )

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)

####################################################################################################################

    # Callback function for the service
    def ik_solver_callback(self, request, response):
        self.get_logger().info(f"Received position x={request.x_coord} y={request.y_coord} z={request.z_coord}")
        response.joint_angles = self.inverse_kinematics_solution(request.x_coord/1000, request.y_coord/1000, request.z_coord/1000, request.gripper_state)
        # Creating a timer        
        self.timer = self.create_timer(self.wait_sec_between_publish, self.timer_callback)
        self.get_logger().info(f"IK solution: {response.joint_angles}")
        return response

    # JTC publisher callback function
    def timer_callback(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=2)
        trajectory_msg.points.append(point)
        self.publisher_.publish(trajectory_msg)
        print("\nTrajectory Sent !\n")
    
    
    # Function to initialize urdf
    def robot_initialize(self,urdf_file):
        self.robotic_arm = ikpy.chain.Chain.from_urdf_file(urdf_file, active_links_mask=[False, False, True, True, True, True, False])
    

    # Forward Kinematics Solver
    def get_fk_solution(self):
        self.robot_initialize(urdf_file)
        T=self.robotic_arm.forward_kinematics([0] * 7)
        print("\nTransformation Matrix :\n",format(T))
        return T


    # Inverse Kinematics Solver
    def inverse_kinematics_solution(self,x,y,z, gripper):
        self.robot_initialize(urdf_file)
        angles=self.robotic_arm.inverse_kinematics([x,y,z])
        angles = np.delete(angles, [0, 1, 6])
        if gripper:
            angles = np.append(angles, [np.pi/3])
        else:
            angles = np.append(angles, [0])

        self.goal_positions = list(angles) 
        print("\nInverse Kinematics Solution :\n" ,self.goal_positions)
        return self.goal_positions
    

####################################################################################################################
####################################################################################################################
    # def timer_callback(self):

    #     if self.starting_point_ok:

    #         self.get_logger().info(f"Sending goal {self.goals[self.i]}.")

    #         traj = JointTrajectory()
    #         traj.joint_names = self.joints
    #         traj.points.append(self.goals[self.i])
    #         self.publisher_.publish(traj)

    #         self.i += 1
    #         self.i %= len(self.goals)

    #     elif self.check_starting_point and not self.joint_state_msg_received:
    #         self.get_logger().warn(
    #             'Start configuration could not be checked! Check "joint_state" topic!'
    #         )
    #     else:
    #         self.get_logger().warn("Start configuration is not within configured limits!")
    
    def joint_state_callback(self, msg):

        if not self.joint_state_msg_received:

            # check start state
            limit_exceeded = [False] * len(msg.name)
            for idx, enum in enumerate(msg.name):
                if (msg.position[idx] < self.starting_point[enum][0]) or (
                    msg.position[idx] > self.starting_point[enum][1]
                ):
                    self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
                    limit_exceeded[idx] = True

            if any(limit_exceeded):
                self.starting_point_ok = False
            else:
                self.starting_point_ok = True

            self.joint_state_msg_received = True
        else:
            return


# Main function
def main():
    rclpy.init(args=None)
    ik_server_node = IKServerNode()
    rclpy.spin(ik_server_node)
    ik_server_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()