#!/usr/bin/env python

import rospy
from actions.msg import depthGoal, depthAction
import smach

from Depth import Depth

class Sink:

    def __init__(self, smach_StateMachine, NAME, INITIAL_PRESSURE, TASK):
        self.INITIAL_PRESSURE = INITIAL_PRESSURE
        self.name = NAME
        sm_sub = smach.StateMachine(outcomes=['depth_success', 'aborted'])
        with sm_sub:
            depthTask = Depth(self.INITIAL_PRESSURE, 'depth_success')
            depthTask.addDepthAction(smach_StateMachine)
        smach_StateMachine.add(self.name, sm_sub,
                               transitions={'depth_success': TASK})
