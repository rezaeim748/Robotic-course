#!/usr/bin/python3

# ROS
import rospy
from geometry_msgs.msg import Twist
from hw4.srv import humanloc


class Controller:
    def __init__(self) -> None:
        # Use these Twists to control your robot
        self.move = Twist()
        self.move.linear.x = 0.1
        self.freeze = Twist()
        self.exist = True

        self.human_x_center = 0
        self.human_y_center = 0
        self.human_width = 0
        self.human_height = 0
        #!!!!!!!!! it is needed to initialized exactly
        self.frame_x_center = 160

        self.dt = 0.1
        rate = 1/self.dt
        self.r = rospy.Rate(rate)


        # The "p" parameter for your p-controller, TODO: you need to tune this
        self.angular_vel_coef = 0.1

        # TODO: Create a service proxy for your human detection service
        
        # TODO: Create a publisher for your robot "cmd_vel"
        self.pub = rospy.Publisher('/follower/cmd_vel', Twist, queue_size = 1)


    def getHumanLocFromService(self, req):
        #global goal
        rospy.wait_for_service('getHumanLocService')
        try:
            missionService = rospy.ServiceProxy('getHumanLocService', humanloc)
            resp1 = missionService(1)

            if (resp1.x_center != -1):
                self.human_x_center = resp1.x_center
                self.human_y_center = resp1.y_center
                self.human_width = resp1.width
                self.human_height = resp1.height
                self.exist = True
            else: 
                self.exist = False
            
                
            #rospy.loginfo(resp1)
            return resp1
        except rospy.ServiceException as e:
           print("Service call failed: %s"%e)
    
    def run(self) -> None:
        try:
            while not rospy.is_shutdown():
                # TODO: Call your service, ride your robot
                self.getHumanLocFromService("person")
                #rospy.loginfo(self.exist)
                if(self.exist):
                    err = self.frame_x_center - self.human_x_center
                    err = err / 100
                    P = self.angular_vel_coef * err
                    self.move.angular.z = P
                    self.move.linear.x = 0.1
                    self.pub.publish(self.move)
                else:
                    self.pub.publish(self.freeze)

                self.r.sleep()
                

        except rospy.exceptions.ROSInterruptException:
            pass
                

if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    
    controller = Controller()
    controller.run()
    
