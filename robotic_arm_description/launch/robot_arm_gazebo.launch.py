import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource

package_name = "robotic_arm_description"
xacro_file = "robotic_arm.xacro"

def generate_launch_description():
    description_package = os.path.join(get_package_share_directory(package_name))
    
    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value=os.path.join(description_package, "urdf", xacro_file),
        description= "Absolute path to the URDF file"
    )
    
    #
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix(package_name), "share"))
    
    # xacro file path
    xacro_file_path = os.path.join(description_package, "urdf", xacro_file)
    
    # Convert xacro to xml format
    robot_description_config = xacro.process_file(xacro_file_path)
    
    # robot description
    robot_description = robot_description_config.toxml()
    
    # gazebo server
    gazebo_serve = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")),
        
    )
    
    # gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")),
        
    )
    
    ################################################################
    gazebo_node = ExecuteProcess(
    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
    output='screen'
)   #################################################################


    # Robot state publisher node
    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters =[{"robot_description": robot_description}]
    )
    
    # Node to spawn the robot in gazebo
    spawner_node = Node(
        package = "gazebo_ros",
        executable = "spawn_entity.py",
        arguments = ["-entity", "robotic_arm", "-topic", "robot_description"],
    )
    
    
    return LaunchDescription([
        env_var,
        model_arg,
        robot_state_publisher,
        #gazebo_node,
        gazebo_serve,
        gazebo_client,
        spawner_node,
    ])