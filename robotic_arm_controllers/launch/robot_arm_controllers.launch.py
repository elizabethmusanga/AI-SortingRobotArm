import os
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch  import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition


description_package = "robotic_arm_description"
controllers_package = "robotic_arm_controllers"
xacro_file = "robotic_arm.xacro"


def generate_launch_description():
    
    # xacro file path
    xacro_file_path = os.path.join(get_package_share_directory(description_package), "urdf", xacro_file)
    
    # Set simulation to be the default execution when the project is run
    is_sim_arg = DeclareLaunchArgument(
        "is_sim", 
        default_value = 'True',
    )

    # Assign value to is_sim at run time
    is_sim = LaunchConfiguration("is_sim")
    
    # Robot description
    robot_description = ParameterValue(
        Command(
           [
            "xacro",
            xacro_file_path,
            "is_sim:=False",
           ]
        ),
        value_type=str
    )
    

    # Robot state publisher node
    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters =[{"robot_description": robot_description}],
        condition = UnlessCondition(is_sim)
    )
    

    # Controller manager node
    controller_manager = Node(
        package = "controller_manager",
        executable = "ros2_control_node",
        parameters = [       
            {"robot_description": robot_description,
             "use_sim_time": is_sim},
            os.path.join(
                get_package_share_directory("robotic_arm_controllers"),
                "config",
                "robot_arm_controllers.yaml",
            ),
        ],
        condition=UnlessCondition(is_sim),
    )

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
            "controller_manager"
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
        is_sim_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        robot_joints_trajectory_controller_spawner,
        gripper_controller_spawner
    ])