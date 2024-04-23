#!/usr/bin/env python

import rospy
import smach
from smach_ros import SimpleActionState
from actions.msg import timeAction, timeGoal

class Forward:

    def __init__(self, smach_StateMachine, NAME, TIME, TASK):
        self.TIME_VALUE = TIME
        self.name = NAME
        smach_StateMachine.add(self.name, \
          SimpleActionState('surgeServer', \
          timeAction, \
          goal_cb=self.goal_callback), \
          transitions={
            'succeeded': TASK,
            'preempted': self.name,
            'aborted': 'aborted'
          })

    def goal_callback(self, userdata, goal):
        rospy.loginfo("Executing State Forward")
        timeOrder = timeGoal()
        timeOrder.time_setpoint = self.TIME_VALUE
        return timeOrder
