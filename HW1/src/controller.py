#!/usr/bin/python3

import rospy
from hw0.msg import proximity
from hw0.msg import result


pub1 = rospy.Publisher('motor1', result, queue_size=10)
pub2 = rospy.Publisher('motor2', result, queue_size=10)


def callback(message):
    # goalstate is obstacle state
    goalState = 0
    minDistance = 201
    if message.up < minDistance :
        minDistance = message.up
        goalState = 1
    if message.right < minDistance :
        minDistance = message.right
        goalState = 2
    if message.down < minDistance :
        minDistance = message.down
        goalState = 3
    if message.left < minDistance :
        minDistance = message.left
        goalState = 4

    
    
    
    
    global currentState
    global rotation
    global direction

    if currentState == 1:
        if goalState == 1:
            rotation = 180
            direction = "saatgard"
        if goalState == 2:
            rotation = 90
            direction = "padsaatgard"
        if goalState == 3:
            rotation  = 0
            direction = "saatgard"
        if goalState == 4:
            rotation = 90
            direction = "saatgard"
    
    if currentState == 2:
        if goalState == 1:
            rotation = 90
            direction = "saatgard"
        if goalState == 2:
            rotation = 180
            direction = "saatgard"
        if goalState == 3:
            rotation  = 90
            direction = "padsaatgard"
        if goalState == 4:
            rotation = 0
            direction = "saatgard"
        
    if currentState == 3:
        if goalState == 1:
            rotation = 0
            direction = "saatgard"
        if goalState == 2:
            rotation = 90
            direction = "saatgard"
        if goalState == 3:
            rotation  = 180
            direction = "saatgard"
        if goalState == 4:
            rotation = 90
            direction = "padsaatgard"
            
    if currentState == 4:
        if goalState == 1:
            rotation = 90
            direction = "padsaatgard"
        if goalState == 2:
            rotation = 0
            direction = "saatgard"
        if goalState == 3:
            rotation  = 90
            direction = "saatgard"
        if goalState == 4:
            rotation = 180
            direction = "saatgard"

    
    
    rospy.loginfo("currentState : " + str(currentState) + ", obstacleState : " + str(goalState) + ", " + str(rotation) + ", " + direction)

    if goalState == 1:
        currentState = 3
    if goalState == 2:
        currentState = 4
    if goalState == 3:
        currentState = 1
    if goalState == 4:
        currentState = 2
    






    msg = result()
    msg.rotation = rotation
    msg.direction = direction
    pub1.publish(msg)
    pub2.publish(msg)

        

    
        
    
    
    
    
    

def controller():

    # if state is 1, it means up
    # if state is 2, it means right
    # if state is 3, it means down
    # if state is 4, it means left
    global currentState
    global direction
    global rotation
    currentState = 1
    direction = ""
    rotation = 0
    
    rospy.init_node('controller', anonymous=True)
    rospy.Subscriber("distance", proximity, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    
    
        
        
if __name__=="__main__":
    controller()
