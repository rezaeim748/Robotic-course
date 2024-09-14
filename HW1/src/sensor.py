#!/usr/bin/python3

import rospy
from hw0.msg import proximity
import random

def talker():
	pub = rospy.Publisher("distance", proximity, queue_size=10)
	rospy.init_node("sensor", anonymous=True)
	rate = rospy.Rate(0.1) #HZ
	
	while not rospy.is_shutdown():
		msg = proximity()
		msg.up = random.randrange(10, 200)
		msg.down = random.randrange(10, 200)
		msg.left = random.randrange(10, 200)
		msg.right = random.randrange(10, 200)
		rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()
		
		
if __name__=="__main__":
	talker()
