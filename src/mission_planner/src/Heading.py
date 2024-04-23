#! /usr/bin/env python
import rospy
from smach import StateMachine
from actions.msg import headingGoal, headingAction
import smach
from smach_ros import SimpleActionState
import actionlib

class Heading(smach.State):
    def __init__(self, NAME, HEADING, TASK):
        self.HEADING = HEADING
        self.TASK = TASK
        self.name = NAME
        smach.State.__init__(self, outcomes=[self.TASK])

    def execute(self, userdata):
        rospy.loginfo("Executing State Concurrent Heading")
        client = actionlib.SimpleActionClient('headingServer', headingAction)
        client.wait_for_server()
        goal = headingGoal(heading_setpoint=self.HEADING)
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_state()
        rospy.loginfo("Result: %s", result)
        if result == -1:
            return 'HeadingReached'
        elif result == 'preempted':
            return 'aborted'
        elif result == 'aborted':
            return 'aborted'
        else:
            return 'aborted'
