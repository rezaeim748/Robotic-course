#!/usr/bin/python3

import rospy
from hw0.msg import result




def callback(msg):

    rospy.loginfo("direction: " + msg.direction + ", rotation: " + str(msg.rotation))
        
    
    
    
    
    

def listener():

    rospy.init_node('motor2', anonymous=True)
    rospy.Subscriber("motor2", result, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
    
        
        
if __name__=="__main__":
    listener()
