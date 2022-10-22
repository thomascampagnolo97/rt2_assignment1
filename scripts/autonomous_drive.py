#! /usr/bin/env python

import rospy
from final_assignment.srv import GoalCoordinates    # service for autonomous driving
import actionlib
from move_base_msgs.msg import *
from actionlib_msgs.msg import *

max_time = rospy.Duration(30) # maximum duration to reach the set goal

class bcolors:
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

'''
    Function that manages the autonomous driving service, waiting if the target is reached
'''
def menage_auto_drive(request):
    
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


'''
    Function that initialises the autonomous driving service
'''
def autonomous_drive_server():
    # initialize the node
    rospy.init_node('autonomous_driving_controller')
    
    #call the service
    service = rospy.Service('goal_coordinates', GoalCoordinates, menage_auto_drive)
    #print("Service ready")
    rospy.spin()

if __name__=="__main__":
    autonomous_drive_server() 