#!/usr/bin/python3
import random
import rospy
from std_msgs.msg import String 
import math
from ros_tutorial.srv import GetNextDestination, GetNextDestinationResponse

def getDestination(req):
    #rospy.loginfo("in mission node  curr_x: " + str(req.current_x) + "  curr_y: " + str(req.current_y))
    newDest = GetNextDestinationResponse()
    newDest.next_x = random.uniform(-10, 10)
    newDest.next_y = random.uniform(-10, 10)
    while math.sqrt(pow(newDest.next_x - req.current_x, 2)  + pow(newDest.next_y - req.current_y, 2)) < 5:
        newDest.next_x = random.uniform(-10, 10)
        newDest.next_y = random.uniform(-10, 10)
    #rospy.loginfo(" distance: "+ str(math.sqrt(pow(newDest.next_x - req.current_x, 2)  + pow(newDest.next_y - req.current_y, 2))))
    #rospy.loginfo("in mission node  next_x: " + str(newDest.next_x) + "  next_y: " + str(newDest.next_y))
    return newDest

def getNextDestinationServer():
    rospy.init_node('mission_node')
    s = rospy.Service('getNextDestinationService', GetNextDestination , getDestination)
    #rospy.loginfo("ready to get req and sed res")
    rospy.spin()
if __name__ == "__main__":
    getNextDestinationServer()
