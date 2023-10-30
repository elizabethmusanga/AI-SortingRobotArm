import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


package_name = "robotic_arm_description"
xacro_file = "robotic_arm.xacro"

def generate_launch_description():
    
    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value=os.path.join(get_package_share_directory(package_name), "urdf", xacro_file),
        description= "Absolute path to the URDF file"
    )   
    
    # Xacro file path
    xacro_file_path = os.path.join(get_package_share_directory(package_name), "urdf", xacro_file)
    
    # Convert xacro to xml format
    robot_description_config = xacro.process_file(xacro_file_path)
    
    # robot description
    robot_description = robot_description_config.toxml()

    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters =[{"robot_description": robot_description}]
    )
    
    joint_state_publisher_gui = Node(
        package= "joint_state_publisher_gui",
        executable = "joint_state_publisher_gui",
    )
    
    rviz_node = Node(
        package= "rviz2",
        executable="rviz2",
        name="rviz2",
        output = "screen",
        arguments = ["-d", os.path.join(get_package_share_directory(package_name), "rviz", "display.rviz")]
        
    )
    
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])