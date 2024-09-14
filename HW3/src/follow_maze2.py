#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

class PIDController():


    def __init__(self):
        
        rospy.init_node('follow_maze', anonymous=False)
        
        self.p = 0.1
        self.i = 0.011
        self.d = 13
        
        self.dt = 0.005
        self.v = 0.2
        self.min_D = 0.5
        rate = 1/self.dt
        
        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)


    
    def maze_runner(self):
                
        i_integral = 0
        last_theta_error = 0
        move_cmd = Twist()
        move_cmd.angular.z = 0
        move_cmd.linear.x = self.v


        while not rospy.is_shutdown():
            laser_data = rospy.wait_for_message("/scan" , LaserScan)
            front_distance = min(laser_data.ranges[0:5])
            left_distance = min(laser_data.ranges[5:180])  

            if front_distance <= self.min_D :
                new_twist = Twist()
                new_twist.angular.z = -0.2
                new_twist.linear.x = 0.0
                self.cmd_vel.publish(new_twist)
            
            else:
                self.cmd_vel.publish(move_cmd)

                error = left_distance - self.min_D
                i_integral += error * self.dt
                
                P = self.p * error
                I = self.i * i_integral
                D = self.d * (error - last_theta_error)

                move_cmd.angular.z = P + I + D 
                last_theta_error = error
                move_cmd.linear.x = self.v            
                                                

                self.r.sleep()

            

if __name__ == '__main__':
    pidc = PIDController()
    pidc.maze_runner()
