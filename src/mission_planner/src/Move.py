#!/usr/bin/env python

import rospy
import smach
from smach_ros import SimpleActionState
from actions.msg import timeAction
from std_msgs.msg import String

class Move:

    def __init__(self, smach_StateMachine, NAME, TYPE, TIME, TASK):
        self.pub = rospy.Publisher('/vectorThruster/direction', String, queue_size=10)
        self.TIME_VALUE = TIME
        self.name = NAME
        self.type = TYPE
        smach_StateMachine.add(self.name,
                                SimpleActionState('forwardServer',
                                                  timeAction,
                                                  goal_cb=self.goal_callback),
                                transitions={
                                    'succeeded': TASK,
                                    'preempted': self.name,
                                    'aborted': 'aborted'
                                })

    def goal_callback(self, userdata, goal):
        rospy.loginfo("Executing State " + self.type.upper())
        self.pub.publish(String(self.type))
        timeOrder = timeAction()
        timeOrder.time_setpoint = self.TIME_VALUE
        return timeOrder
