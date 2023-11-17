#!/usr/bin/env python3
import numpy as np
import ikpy.chain


urdf_file="/home/newton/ROS2/ai_sorting_robotic_arm/src/AI-SortingRobotArm/robotic_arm_description/urdf/robotic_arm_urdf.urdf"

robotic_arm = ikpy.chain.Chain.from_urdf_file(urdf_file, active_links_mask= [False, False, True, True, True, True, False, False], last_link_vector= np.array([-0.02875, 0.179419, -0.295562]))
T=robotic_arm.forward_kinematics([0]* 8)
print("++++++++++++++++++++++++++++++++++++++++++++++++\n")

# for link in robotic_arm.links:
#     print(f"{robotic_arm.links}")

print(f"joints = {len(robotic_arm.links)}")
print("\n")
print("++++++++++++++++++++++++++++++++++++++++++++++++\n")
print("\nTransformation Matrix :\n",T)
angles=robotic_arm.inverse_kinematics([-0.1, -0.01, 0.3])
print("\nAngles Computed\n",angles)
angles=np.delete(angles, [0, 1, 6, 7])
print("\nCorrected Angles \n",list(angles)) 