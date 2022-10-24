#! /usr/bin/env python
"""
.. module:: manual_drive
    :platform: Unix
    :synopsis: Python module to move the robot manually with the keyboard of the node `teleop_twist_keyboard`
   
.. moduleauthor:: Thomas Campagnolo <thomascampagnolo.s5343274@gmail.com>

This node implements the lauching of the two manual driving modalities:
   1. the user is able to drive the robot without any constraint, full manual driving experience;
   2. the user drives the robot but there will be a collision control avoiding the user to hit obstacles, assisted manual driving experience.

Service:
    /manual_drive
"""


import rospy
import os
from final_assignment.srv import ManualDrive    # service for manual driving	

class bcolors:
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
   
def menage_manual_drive(request):
    """
    Function that manages the manual driving service, called by both 
    full and assisted manual driving mode choosen the launcher the specific mode.

    Args:
        request (Int32): manual driving modality selected by the user

    The user choice is passed to the `os` to launch the chosen launch file.
    """
    # full manual driving without assisted help to avoid obstacles
    if request.manual_driving_mode == 1:
       print(f"{bcolors.WARNING}Prepare for manual driving without assistance{bcolors.ENDC}\n")
       # launch the teleop_twist_keyboard node to drive the robot in the environment
       os.system("roslaunch final_assignment full_manual_drive.launch") 
    
    # assisted manual driving to avoid obstacles
    elif request.manual_driving_mode == 2:
        print(f"{bcolors.WARNING}Prepare for manual driving with obstacle avoidance control{bcolors.ENDC}\n")
        # launch the teleop_twist_keyboard node and the osbstacle avoidance 
        # to drive the robot in the environment
        os.system("roslaunch final_assignment assisted_manual_drive.launch")
    else:
        print("Wrong input!")
    return 0         

def manual_drive_server():
    """
    Function that initialises the manual driving service

    Uses the service handler to manage the `ManualDrive` service, returning the values 
    from the `menage_manual_drive` function.
    """

    #initialize the node
    rospy.init_node('manualDrive_controller')
    
    #call the service
    service = rospy.Service('manual_drive', ManualDrive, menage_manual_drive)
    #print("Service ready!")
    rospy.spin()


if __name__=="__main__":
    manual_drive_server()