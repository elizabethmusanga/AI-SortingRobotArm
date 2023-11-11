#!/usr/bin/env python3
import os
import ikpy.chain

urdf_file="/home/newton/ROS2/ai_sorting_robotic_arm/src/AI-SortingRobotArm/robotic_arm_description/urdf/robotic_arm_urdf.urdf"

robotic_arm = ikpy.chain.Chain.from_urdf_file(urdf_file)
T=robotic_arm.forward_kinematics([0]* 7)
print("++++++++++++++++++++++++++++++++++++++++++++++++\n")

for link in robotic_arm.links:
    print(f"{robotic_arm.links}")
    
print("\n")
print("++++++++++++++++++++++++++++++++++++++++++++++++\n")
print("\nTransformation Matrix :\n",T)
angles=robotic_arm.inverse_kinematics([-0.2 , -0.2, 0.4])
print("\nAngles Computed\n",angles)
# angles=np.delete(angles, [0, 4, 6, 7])
print("\nCorrected Angles \n",list(angles)) 