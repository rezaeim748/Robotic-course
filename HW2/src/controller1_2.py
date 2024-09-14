#!/usr/bin/python3

#final controller for part 1


import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from ros_tutorial.msg import position
from ros_tutorial.srv import GetNextDestination
from math import atan2

x = 0.0
y = 0.0 
theta = 0.0
GO, ROTATE = 0, 1
state = ROTATE
firstRot = 0
iteration = 0
distanceSum = 0


rospy.init_node("speed_controller")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
speed = Twist()
linearSpeed = rospy.get_param("/controller/linear_speed") # m/s

goal = Point()
goal.x = 5
goal.y = 5
start = Point()
start.x = 0
start.y = 0
lastDistance = 30

sum_i_theta = 0
prev_theta_error = 0

k_i = 0
k_p = 0.6
k_d = 7
        
dt = 0.005
v = 0.6
D = 2

rate = 1/dt
r = rospy.Rate(rate)
errs = []










def newPos():
    msg = rospy.wait_for_message("/odom" , Odometry)
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def getDestinationFromService(x, y):
        global goal
        rospy.wait_for_service('getNextDestinationService')
        try:
           missionService = rospy.ServiceProxy('getNextDestinationService', GetNextDestination)
           resp1 = missionService(x, y)
           goal.x = resp1.next_x
           goal.y = resp1.next_y
           rospy.loginfo(resp1)
           return resp1
        except rospy.ServiceException as e:
           print("Service call failed: %s"%e)


def distance_from_wall():
        laser_data = rospy.wait_for_message("/scan" , LaserScan)
        rng = laser_data.ranges[:180]
        d = min(rng)
        return d

#d = distance_from_wall()




#def update_next(msg: GetNextDestination):
#    global goal
#    global firstRot
#    global state
#    global ROTATE
#    global iteration
#
#    if firstRot == 0 and state == ROTATE:
#        goal.x = msg.next_x
#        goal.y = msg.next_y
#        iteration = iteration + 1
#        rospy.loginfo(msg)
#        #"goal.x : $s" ,
#    firstRot = 1

#sub = rospy.Subscriber("getNextDestinationService" , GetNextDestination )











while not rospy.is_shutdown():
    if iteration > 4:
        rospy.loginfo("mean distance:   " + str(distanceSum / 5))
        speed.linear.x = 0.000000
        speed.angular.z = 0.000000
        pub.publish(speed)
        break

    inc_x = goal.x -x
    inc_y = goal.y -y

    angle_to_goal = atan2(inc_y, inc_x)
    #rospy .loginfo(angle_to_goal - theta)
    newPos()

    if state == ROTATE:
        if abs(angle_to_goal - theta) > 0.1 :
            speed.linear.x = 0.000000
            speed.angular.z = 0.300000
            state = ROTATE
        else:
            speed.linear.x = 0.000000
            speed.angular.z = 0.000000
            pub.publish(speed)
            r.sleep()
            r.sleep()
            r.sleep()
            r.sleep()
            state = GO
    #elif abs(angle_to_goal - theta) < 0.1 and state == ROTATE:
    #    pub.publish(Twist())
    else:
        e1 = ((goal.x - start.x) * (start.y - y)) - ((start.x - x) * (goal.y - start.y))
        e2 = pow((pow(goal.x - start.x, 2) + pow(goal.y - start.y, 2)), 0.5)
        err = e1 / e2
        errs.append(err)
        sum_i_theta += err * dt
            
        P = k_p * err
        I = k_i * sum_i_theta
        D = k_d * (err - prev_theta_error)

        #rospy.loginfo(f"P : {P} I : {I} D : {D}")
        speed.angular.z = P + I + D 
        prev_theta_error = err
        speed.linear.x = v            
            
        rospy.loginfo(f"error : {err} speed : {speed.linear.x} theta : {speed.angular.z}")
            
        #d = distance_from_wall()



        #speed.linear.x = linearSpeed
        #speed.angular.z = 0.000000
        state = GO
        distance = ((goal.x - x) ** 2 + (goal.y - y) ** 2) ** 0.5
        #rospy.loginfo("distance to goal:   " + str(distance))
        if distance < 1 or distance > lastDistance:
            distanceSum = distanceSum + distance
            state = ROTATE
            firstRot = 0
            getDestinationFromService(x, y)
            start.x = x
            start.y = y
            lastDistance = 30
            iteration = iteration + 1
            #rospy.loginfo("//////////////////////")
            #rospy.loginfo(x)
            #rospy.loginfo(y)
            r.sleep()
            continue
        lastDistance = distance



    pub.publish(speed)
    r.sleep()    