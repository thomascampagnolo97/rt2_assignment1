#! /usr/bin/env python
"""
.. module:: autonomous_drive
    :platform: Unix
    :synopsis: Python module to move the robot autonomously to the coordinates given by the user 
   
.. moduleauthor:: Thomas Campagnolo <thomascampagnolo.s5343274@gmail.com>

This node implements the automatic driving mode of the robot from it's current position, to the one 
inserted by the user. It's also implemented a time-condition to evaluate if the desired position
is reachable or not.

Service:
    /goal_coordinates

Action:
    /move_base
    /actionlib
"""


import rospy
from final_assignment.srv import GoalCoordinates    # service for autonomous driving
import actionlib
from move_base_msgs.msg import *
from actionlib_msgs.msg import *

max_time = rospy.Duration(30) # maximum duration to reach the set goal
"""
Maximum duration to reach the set goal
"""

class bcolors:
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'


def menage_auto_drive(request):
    """
    Function that manages the autonomous driving service, waiting an appropriate time to check 
    if the target is reached.

    Args:
        request (Float64 x, Float64 y): goal coordinates coming from the service

    Returns:
        1: if the target is reached successfully
        -1: if the target can't be reached and the goal has canceled
    """
    
    x = request.x
    y = request.y

    print("Going to point x: ", x," y: ", y)
    
    # starting the action and wait for the server 
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    
    # set the target's parameters
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.orientation.w = 1
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    
    # send the target goal to the client
    client.send_goal(goal)
    # Execution time to prevent the robot from continuing to move 
    # without reaching the terget
    timeout = client.wait_for_result(max_time)
    if not timeout:
        # the target is not reached
        print(f"{bcolors.FAIL}\nThe target can't be reached!{bcolors.ENDC}")
        client.cancel_goal()
        print(f"{bcolors.FAIL}\n\n!!! Goal has been canceled !!!\n\n{bcolors.ENDC}")
        return -1

    # the target is reached
    print(f"{bcolors.OKGREEN}\nArrived at destination.{bcolors.ENDC}")
    return 1

def autonomous_drive_server():
    """
    Function that initialises the autonomous driving service.

    Uses the service handler to manage the `GoalCoordinates` service, returning the values 
    from the `menage_auto_drive` function.
    """

    # initialize the node
    rospy.init_node('autonomous_driving_controller')
    
    #call the service
    service = rospy.Service('goal_coordinates', GoalCoordinates, menage_auto_drive)
    #print("Service ready")
    rospy.spin()

if __name__=="__main__":
    autonomous_drive_server() 