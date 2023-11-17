import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robotic_arm_description_dir = get_package_share_directory('robotic_arm_description')

    model_arg = DeclareLaunchArgument(name='model', 
                                      default_value=os.path.join(
                                                    robotic_arm_description_dir, 'urdf', 'robotic_arm.xacro'
                                                    ),
                                      description='Absolute path to robot urdf file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)



    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(robotic_arm_description_dir, 'rviz', 'display.rviz')],
    )

    return LaunchDescription([
        model_arg,
        # joint_state_publisher_gui_node,
        # robot_state_publisher_node,
        rviz_node
    ])
