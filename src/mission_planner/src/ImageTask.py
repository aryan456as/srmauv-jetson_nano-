#! /usr/bin/env python
import rospy
import smach

import numpy as np
import cv2
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

from Depth import Depth
from Heading import Heading
# from darknet_ros_msgs.msg import BoundingBox
from std_msgs.msg import Float64
from sensor_msgs.msg import Image

class ImageTask():
    def __init__(self, TASK):
        self.bridge = CvBridge()
        self.TASK = TASK
        self.depth_pub = rospy.Publisher('/depth_setpoint', Float64, queue_size=10)
        self.heading_pub = rospy.Publisher('/heading_setpoint', Float64, queue_size=10)
        self.imagePub = rospy.Publisher('/buoy', Image, queue_size=1)
        rospy.Subscriber('/camera/image_raw', Image, self.callback)

        sm_sub = smach.Concurrence(
            outcomes=['DepthHeadingReached', 'DepthHeadingFailed'],
            default_outcome='DepthHeadingFailed',
            outcome_map={
                'DepthHeadingReached': {'DEPTH_CONCURRENT': 'DepthReached', 'HEADING_CONCURRENT': 'HeadingReached'}
            }
        )

        with sm_sub:
            smach.Concurrence.add('DEPTH_CONCURRENT', Depth(self.x, 'depth_success'))
            smach.Concurrence.add('HEADING_CONCURRENT', Heading(self.y, 'heading_success'))

        smach.StateMachine.add('IMAGETASK', sm_sub, transitions={
            'DepthHeadingFailed': 'IMAGETASK',
            'DepthHeadingReached': self.TASK
        })

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # Processing image
        # Perform your image processing here
        
        try:
            self.imagePub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

    def startImageTask(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('image_task_node')
    image_task = ImageTask('some_task')
    image_task.startImageTask()
