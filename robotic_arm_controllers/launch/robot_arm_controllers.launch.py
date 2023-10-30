import os
import xacro
from launch_ros.actions import Node
from launch  import LaunchDescription
from ament_index_python.packages import get_package_share_directory

description_package = "robotic_arm_description"
controllers_package = "robotic_arm_controllers"
xacro_file = "robotic_arm.xacro"


def generate_launch_description():
    
    # xacro file path
    xacro_file_path = os.path.join(get_package_share_directory(description_package), "urdf", xacro_file)
    
    
    
    
    # joint_state_broadcaster node
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
        ]
    )
    
    # arm_controller
    robot_joints_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robot_joints_joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    
    # gripper controller node
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "controller_manager"
        ]
    )
    
    
    return LaunchDescription([
        joint_state_broadcaster_spawner,
        robot_joints_trajectory_controller_spawner,
        gripper_controller_spawner
    ])