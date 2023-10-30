#!/usr/bin/env python3
import ikpy.chain
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

# urdf file
urdf_file=os.path.join("/home/newtonjeri/ai_based_sorting_robot_arm/src/robotic_arm_description" ,"urdf","robotic_arm_urdf.urdf")

robotic_arm = ikpy.chain.Chain.from_urdf_file(urdf_file, active_links_mask=[False, False, True, True, True, True, False])
T=robotic_arm.forward_kinematics([0]* 7)
print("++++++++++++++++++++++++++++++++++++++++++++++++\n")
#print(f"{robotic_arm.links}")
print("\n")
print("++++++++++++++++++++++++++++++++++++++++++++++++\n")
print("\nTransformation Matrix :\n",T)
angles=robotic_arm.inverse_kinematics([-0.7 , -0.1, 0.4])
print("\nAngles Computed\n",angles)
# angles=np.delete(angles, [0, 4, 6, 7])
print("\nCorrected Angles \n",list(angles)) 