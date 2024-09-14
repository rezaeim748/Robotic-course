#!/usr/bin/python3

# Python
import copy

# Object detection
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator
from ultralytics.yolo.engine.results import Results
import torch
import os
from hw4.srv import humanloc, humanlocResponse

# ROS
import rospy
from sensor_msgs.msg import Image


class ImageProcessor:
    def __init__(self) -> None:

        self.counter = 0


        # Image message
        #path = os.path.join(os.path.expanduser('~'), 'Desktop', 'catkin_ws', 'src', 'hw4', 'src', '2.jpg' )
        self.image_msg = Image()

        self.image_res = 240, 320, 3 # Camera resolution: height, width
        self.image_np = np.zeros(self.image_res) # The numpy array to pour the image data into

        # TODO: Subscribe on your robot's camera topic
        # NOTE: Make sure you use the provided listener for this subscription
        self.camera_subscriber = rospy.Subscriber("/follower/camera/image", Image, self.camera_listener)

        # TODO: Instantiate your YOLO object detector/classifier model
        self.model = YOLO('../yolo/yolov8n.pt')
        #self.model = YOLO('../yolo/yolov8n.pt')
        # TODO: You need to update results each time you call your model
        self.results = self.model(self.image_np)

        self.cv2_frame_size = 400, 320
        cv2.namedWindow("robot_view", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robot_view", *self.cv2_frame_size)

        # TODO: Setup your "human detection" service
        self.human_detection_server = rospy.Service('getHumanLocService', humanloc , self.get_human_loc)
        #self.human_detection_server = rospy.Service('human_detection_Service', GetNextDestination , self.get_human_loc)

        self.update_view()


    def camera_listener(self, msg: Image):
        self.image_msg.data = copy.deepcopy(msg.data)

    
    def get_human_loc(self, req):
        boxes = self.results[0].boxes
        response = humanlocResponse()
        response.x_center = -1
        dict = {}
        x=0
        # itterate on boxes and set the result x, w, h, flage of existance
        for b in boxes:
            # set variables
            pList = b.xyxy[0].tolist()
            response.x_center = round((pList[0] + pList[2]) / 2)
            #rospy.loginfo(response.x_center)
            h = self.image_res[0]
            w = self.image_res[1]
            dict[self.results[0].names[b.cls[0].item()]] = h
        

        #rospy.loginfo(response)
        return response


    
      
      
      
      


    def update_view(self):
        try:
            while not rospy.is_shutdown():
                if len(self.image_msg.data) == 0: # If there is no image data
                    continue

                # Convert binary image data to numpy array
                self.image_np = np.frombuffer(self.image_msg.data, dtype=np.uint8)
                self.image_np = self.image_np.reshape(self.image_res)

                frame = copy.deepcopy(self.image_np)

                # TODO: You can use an "Annotator" to draw object bounding boxes on frame
                self.results = self.model(self.image_np, verbose = False)

                cv2.imshow("robot_view", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)

        except rospy.exceptions.ROSInterruptException:
            pass


if __name__ == "__main__":
    rospy.init_node("image_processor", anonymous=True)

    rospy.on_shutdown(cv2.destroyAllWindows)

    image_processor = ImageProcessor()

    

    rospy.spin()

