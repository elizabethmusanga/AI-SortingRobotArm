#!/usr/bin/env python3

import json

data = ' {"joint_1":0,"joint_2":0,"joint_3":30,"joint_4":0} '

joint_angles = json.loads(data)


print(f"Jointangles = {joint_angles}\n")


def assign_values(joint_angles:dict):
    joint_1 = 1.250
    joint_2 = 1.001
    joint_3 = 1.245
    joint_4 = 0.122

    joint_angles["joint_1"]= joint_1
    joint_angles["joint_2"]= joint_2
    joint_angles["joint_3"]= joint_3
    joint_angles["joint_4"]= joint_4
    
    return joint_angles
    

print(f"Jointangles = {assign_values(joint_angles)}\n")