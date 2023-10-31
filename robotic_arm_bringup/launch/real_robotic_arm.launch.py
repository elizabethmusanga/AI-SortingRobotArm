import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    controllers = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("robotic_arm_controllers"),
                "launch",
                "robot_arm_controllers.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    moveit = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("robotic_arm_moveit"),
                "launch",
                "moveit.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    remote_interface = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("robotic_arm_remote"),
                "launch",
                "remote_interface.launch.py"
            ),
        )
    
    return LaunchDescription([
        controllers,
        moveit,
        remote_interface,
    ])