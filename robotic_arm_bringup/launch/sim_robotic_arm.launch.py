import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("robotic_arm_description"),
                "launch",
                "robot_arm_gazebo.launch.py"
            )
        )
    
    controllers = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("robotic_arm_controllers"),
                "launch",
                "robot_arm_controllers.launch.py"
            ),
            launch_arguments={"sim_mode": "True"}.items()
        )
    
    rviz = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robotic_arm_description"),
            "launch",
            "robot_arm_display.launch.py"
        ),
        launch_arguments={"sim_mode": "True"}.items()
    )
    
    
    camera = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("robotic_arm_camera"),
                "launch",
                "robot_arm_camera.launch.py"
            ),
        )
    
    kinematics = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("robotic_arm_kinematics"),
                "launch",
                "robotic_arm_kinematics.launch.py"
            ),
        )
    
    return LaunchDescription([
        rviz,
        gazebo,
        camera,
        controllers,
        kinematics
    ])