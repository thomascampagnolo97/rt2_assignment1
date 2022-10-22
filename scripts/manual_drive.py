#! /usr/bin/env python

import rospy
import os
from final_assignment.srv import ManualDrive    # service for manual driving	

class bcolors:
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
   
'''
    Function that manages the manual driving service, called by both 
    full and assisted manual driving mode choosen the launcher the specific mode.
'''
def menage_manual_drive(request):
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

'''
    Function that initialises the manual driving service
'''  
def manual_drive_server():
    #initialize the node
    rospy.init_node('manualDrive_controller')
    
    #call the service
    service = rospy.Service('manual_drive', ManualDrive, menage_manual_drive)
    #print("Service ready!")
    rospy.spin()


if __name__=="__main__":
    manual_drive_server()