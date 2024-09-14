#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from cmath import rect
import numpy as np
import rospy
import tf
import math
from math import atan2, pi, radians, sqrt



class PIDController():


    def __init__(self):
        
        rospy.init_node('wall_follower', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        
        self.k_i_angle = 0.03
        self.k_p_angle = 3
        self.k_d_angle = 0.05

        self.k_i_distance = 0.0
        self.k_p_distance = 10
        self.k_d_distance = 0.5


        self.speed = Twist()        
        self.dt = 0.005
        self.v = 0.6
        #self.D = 2
        rate = 1/self.dt
        
        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.errs = []

        star = []
        X1 = np.linspace(0, 3 , 100)
        Y1 = - (7/3) * X1  + 12
        star = [*star, *[(i, j) for (i, j) in zip(X1, Y1)]]

        X2 = np.linspace(3, 10 , 100)
        Y2 = np.array([5]*100)
        star = [*star, *[[i, 5.0] for i in X2]]

        X3 = np.linspace(10, 4 , 100)
        Y3 = (5/6) * X3  - (10/3)
        star = [*star, *[(i, j) for (i, j) in zip(X3, Y3)]]


        X4 = np.linspace(4, 7 , 100)
        Y4 = -(3) * X4  + 12
        star = [*star, *[(i, j) for (i, j) in zip(X4, Y4)]]


        X5 = np.linspace(7, 0 , 100)
        Y5 = -(4/7) * X5  - 5
        star = [*star, *[(i, j) for (i, j) in zip(X5, Y5)]]


        X6 = np.linspace(0, -7 , 100)
        Y6 = (4/7) * X6  - 5
        star = [*star, *[(i, j) for (i, j) in zip(X6, Y6)]]


        X7 = np.linspace(-7, -4 , 100)
        Y7 = 3 * X7  + 12
        star = [*star, *[(i, j) for (i, j) in zip(X7, Y7)]]


        X8 = np.linspace(-4, -10 , 100)
        Y8 = -(5/6) * X8  - (10/3)
        star = [*star, *[(i, j) for (i, j) in zip(X8, Y8)]]


        X9 = np.linspace(-10, -3 , 100)
        Y9 = np.array([5]*100)
        star = [*star, *[[i, 5.0] for i in X9]]

        X10 = np.linspace(-3, 0 , 100)
        Y10 = (7/3) * X10  + 12
        star = [*star, *[(i, j) for (i, j) in zip(X10, Y10)]]
        
        self.path = star


    def distance_from_wall(self):
        laser_data = rospy.wait_for_message("/scan" , LaserScan)
        rng = laser_data.ranges[:180]
        d = min(rng)
        return d

    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        _, _, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        return yaw

    
    def follow_wall(self):
        
        d = self.distance_from_wall()    
        sum_i_theta = 0
        prev_theta_error = 0
        
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.v


        while not rospy.is_shutdown():
            last_angle = 0
            for[x, y] in self.path:
                self.cmd_vel.publish(Twist())

                msg = rospy.wait_for_message("/odom" , Odometry) 
                curr_x = msg.pose.pose.position.x
                curr_y = msg.pose.pose.position.y

                dx = x - curr_x
                dy = y - curr_y

                distance = (dx ** 2 + dy ** 2)

                sum_of_distance = 0
                sum_of_angle = 0
                last_distance = 0

                while distance > 0.1:

                    curr_angle = self.get_heading()
                    msg = rospy.wait_for_message("/odom" , Odometry) 
                    curr_x = msg.pose.pose.position.x
                    curr_y = msg.pose.pose.position.y

                    self.errs.append(sqrt(min([((curr_x - i) ** 2 + (curr_y - j) ** 2) for [i, j] in self.path])))
                    dx = x - curr_x
                    dy = y - curr_y
                    path_angle = atan2(dy , dx)

                    if last_angle > pi-0.1 and curr_angle <= 0:
                        curr_angle = 2*pi + curr_angle
                    elif last_angle < -pi+0.1 and curr_angle > 0:
                        curr_angle = -2*pi + curr_angle
                        
                    if path_angle < -pi/4 or path_angle > pi/4:
                        if y < 0 and curr_y < y:
                            path_angle = -2*pi + path_angle
                        elif y >= 0 and curr_y > y:
                            path_angle = 2*pi + path_angle

                    distance = sqrt(dx**2 + dy**2)
                    var_distance = distance - last_distance
                    control_distance = self.k_p_distance*distance + self.k_i_distance*sum_of_distance + self.k_d_distance*var_distance
                    control_angle = self.k_p_angle*(path_angle - curr_angle)

                    self.speed.angular.z = (control_angle)
                    self.speed.linear.x = min(control_distance,0.1)
                    if self.speed.angular.z > 0:
                        self.speed.angular.z = min(self.speed.angular.z, 1.5)
                    else:
                        self.speed.angular.z = max(self.speed.angular.z, -1.5)

                    last_angle = curr_angle
                    self.cmd_vel.publish(self.speed)

                    sum_of_distance += distance
                    last_distance = distance
                    sum_of_angle += path_angle

            

            #self.r.sleep()

    def on_shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        plt.plot(list(range(len(self.errs))),
                    self.errs, label='errs')
        plt.axhline(y=0,color='R')
        plt.draw()
        plt.legend(loc="upper left", frameon=False)
        #plt.savefig(f"errs_{self.k_p}_{self.k_d}_{self.k_i}.png")
        plt.show()

        rospy.sleep(1)
            

if __name__ == '__main__':
    try:
        pidc = PIDController()
        pidc.follow_wall()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")