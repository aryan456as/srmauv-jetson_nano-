#!/usr/bin/env python

import rospy
import smach
from smach_ros import SimpleActionState
import actions.msg
from std_msgs.msg import String

class Sway:

    def __init__(self, smach_StateMachine, NAME, TIME, TASK):
        self.TIME_VALUE = TIME
        self.name = NAME
        smach_StateMachine.add(self.name, \
          SimpleActionState('swayServer', \
          actions.msg.timeAction, \
          goal_cb=self.goal_callback), \
          transitions={
            'succeeded': TASK,
            'preempted': self.name,
            'aborted': 'aborted'
          })

    def goal_callback(self, userdata, goal):
        rospy.loginfo("Executing State Sway")
        timeOrder = actions.msg.timeGoal()
        timeOrder.time_setpoint = self.TIME_VALUE
        return timeOrder
