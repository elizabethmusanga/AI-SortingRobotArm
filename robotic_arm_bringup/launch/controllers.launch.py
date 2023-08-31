import os
import xacro
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

description_pkg_name = 'robotic_arm_description'
configuration_file = 'controllers_configuration.yaml'

def generate_launch_description():
    package_share_dir = get_package_share_directory(description_pkg_name)
    
    xacro_file = os.path.join(package_share_dir, 'urdf', 'robotic_arm.xacro')
    assert os.path.exists(
        xacro_file), "The robot.xacro doesnt exist in "+str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()
    
    robot_description = {"robot_description": robot_desc}
    
    # Load ros2_control_node
    controller_manager = Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, configuration_file],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            )
    
    #joint_state_broadcaster_controller_node,
    joint_state_broadcaster_controller_node = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
            )

    #joint_trajectory_controller_node,
    joint_trajectory_controller_node = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            )
    
    return LaunchDescription(
        [
            controller_manager, 
            joint_state_broadcaster_controller_node,
            joint_trajectory_controller_node,
        ]
    )