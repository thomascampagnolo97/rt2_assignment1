#! /usr/bin/env python
"""
.. module:: user_interface_controller
   :platform: Unix
   :synopsis: Python module for the User Interface Controller
   
.. moduleauthor:: Thomas Campagnolo <thomascampagnolo.s5343274@gmail.com>

This is the main ROS node about the final assignment of the Research Track 1 course of the 
Robotics Engineering Master Degree.

**Overview**

The simulation involves a mobile robot positioned inside an environment, and it's equipped 
with a laser scanner in order to detect the presence of obstacles, such as the walls of the 
environment. The aim of the project is to develop an architecture to control the robot in the environment. 

The software will rely on the move_base and gmapping packages for localizing the robot and plan the motion. 

The implemented architecture allows the user to perform three different robot behavior modes, in particular:
    1. Reach autonomously the coordinate (x, y) entered by the user, thus obtaining an autonomous guide of the robot;
    2. Let the user to drive the robot with the keyboard, thus having the opportunity to try a manual robot driving experience;
    3. Let the user to guide the robot with the keyboard with assistance to avoid collisions with obstacles.

In addition, the user interface allows to reset the position of the robot to the initial state or to exit 
the program by closing all active nodes.

Service:
    /goal_coordinates to set the goal coordinates that the robot has to reach autonomously
    /manual_drive to drive manually the robot with the keyboard
    /gazebo/reset_world to reset the position of the robot to the initial state
"""


import rospy
import os

from std_srvs.srv import Empty
from final_assignment.srv import GoalCoordinates    # service for autonomous driving
from final_assignment.srv import ManualDrive        # service for manual driving

class bcolors:
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'


def user_interface():
    """
    Function to print the user interface with the available driving modes on the screen.

    Returns:
        user_input (Int): user's input, value of the selected mode.
    """

    print("***************************************************************************************")
    print('Hello! Please select between the different modalities to decide the robot driving mode:')
    print('1 ---> Autonomus drive, setting a goal point using (x,y) coordinates')
    print('2 ---> Manual driving experience, using the keyboard to control the robot')
    print('9 ---> Reset robot position')
    print('0 ---> EXIT\n')
    print("***************************************************************************************")

    while True:
        try:
            # if input is an integer it exit from the while
            user_input = int(input('Modality: '))
            break
        except:
            print('Please, type an integer number\n')
    
    return user_input


def autonomous_drive():
    """
    Function for autonomous driving of the robot: the user sets the goal coordinates 
    and checks whether the set goal is reached.

    Service:
        /goal_coordinates
    
    The user goal coordinates (x,y) is passed to the service ``GoalCoordinates``, advertised by :mod:`autonomous_drive`.
    """

    print("Modality selected 1: Autonomus drive\n")
    x = float(input("Insert x-coordinate: "))
    y = float(input("Insert y-coordinate: "))

    # calls the service GoalCoordinates and sends the coordinates to the menage_auto_drive 
    # function in autonomous_drive.py
    rospy.wait_for_service('goal_coordinates')
    goal_coordinates = rospy.ServiceProxy('goal_coordinates', GoalCoordinates)
    goal = goal_coordinates(x,y)
    
    # Check the target value
    if goal.return_== 1:
        # Goal achieved
    	print(f"{bcolors.OKGREEN}Target reached successfully!{bcolors.ENDC}\n")
    else:
        # Goal not achieved
    	print(f"{bcolors.FAIL}Target not reached!{bcolors.ENDC}\n")

       	
def manual_drive():
    """
    Function for manual driving of the robot: calls the service to manage the 
    input from keyboard.

    Service:
        /manual_drive
    
    The user can select the type of driving:
    1. Full manual driving experience, or
    2. Assisted manual driving experience with obstacle avoidance.
    
    The type is passed to the service ``ManualDrive``, advertised by :mod:`manual_drive`.
    """

    #if the user selects mode 2 it will 
    
    print("Modality selected 2: Manual drive\n")

    # calls the service ManualDrive
    rospy.wait_for_service('manual_drive')
    manual_driving = rospy.ServiceProxy('manual_drive', ManualDrive)

    print('\nSelect the type of driving')
    print('1: Full manual driving experience')
    print('2: Assisted manual driving experience\n\n')

    while True:
        try:
            # if input is an integer it exit from the while
            user_manual_mode = int(input('Type of driving: '))
            break
        except:
            print('\nIncorrect input. Please, type one of the two values described above\n')

    # Check the type of manual driving
    if user_manual_mode == 1:
        # send 1 to the manual_driving_mode (request) of the service ManualDrive, called in manual_drive.py.
        # Full manual driving experience, without assistance to avoid obstacles
        manual_driving(1)
    elif user_manual_mode == 2:
        # send 2 to the manual_driving_mode (request) of the service ManualDrive, called in manual_drive.py
        # Assisted manual driving experience to avoid obstacles
        manual_driving(2)


def driving_modality(user_input):
    """
    Function used to decide the behavior of the robot according to the user input

    Args:
        user_input (Int): user's input of the selected driving modes.
    """

    if user_input == 1:
        # Autonomus driving with goal set from user
        autonomous_drive()
    elif user_input == 2:
        # Manual driving experience using the keyboard
        manual_drive()
    elif user_input == 9:
        # Reset robot position
        reset_world()
        os.system('clear')
        print(f"{bcolors.WARNING}Reset environment!{bcolors.ENDC}\n")   # Feedback to user
    # Exit loop and shutdown ros
    elif user_input == 0:
        # Clear terminal
        os.system('clear')
        print(f"{bcolors.WARNING}Programs are EXITING{bcolors.ENDC}\n\n")
        # Kill all the nodes       
        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")
        for node in nodes:
            os.system("rosnode kill "+ node)
        os.system("killall -9 rosmaster")

    else:
        # None of the options available 
        # Clear terminal
        os.system('clear')
        print(f"{bcolors.WARNING}Invlid input.{bcolors.ENDC}\n\n")


def main():
    """
    Main function of the program which starts the service for resetting the gazebo 
    environment and the user interface for selecting the robot driving modalities.
    """

    global reset_world
    """
    Global service for reset the robot position.
    """

    #initialize the ros node of the user interface
    rospy.init_node('user_interface_controller')

    # Create a client to reset the simulation environment
    rospy.wait_for_service('/gazebo/reset_world')
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)

    # Loop user interface
    while not rospy.is_shutdown():
        # Get the driving modality from user
        user_input = user_interface()
        # The selected driving mode starts
        driving_modality(user_input)

 
if __name__=="__main__":
    main()
