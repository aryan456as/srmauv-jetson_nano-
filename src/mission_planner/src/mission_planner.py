#!/usr/bin/env python

import rospy
from smach import StateMachine
import smach
from smach_ros import IntrospectionServer
from Sink import Sink
from Heading import Heading
from Forward import Forward
import time
from Sway import Sway
from DetectBuoy import DetectBuoy
from std_msgs.msg import Float64

class MissionPlanner:
    def __init__(self):
        self.boolean = False
        rospy.init_node('mission_planner')
        rospy.Subscriber("/zsys", Float64, self.callback)
        rospy.spin()

    def main(self):
        sm = smach.StateMachine(outcomes=['mission_complete', 'mission_failed', 'aborted'])

        with sm:
            Sink(sm, 'SINK1', 515, 'HEADING1')
            Heading(sm, 'HEADING1', 75, 'FORWARD1')
            Forward(sm, 'FORWARD1', 6, 'FORWARD2')
            Forward(sm, 'FORWARD2', 12, 'mission_complete')

        sis = IntrospectionServer('ZARNA_MISSION_PLANNER', sm, '/START_ZARNA')
        sis.start()
        outcome = sm.execute()
        self.boolean = True
        sis.stop()
        rospy.loginfo("Mission Complete")

    def callback(self, data):
        rospy.loginfo(data.data)
        if data.data == 1 and not self.boolean:
            self.main()

if __name__ == '__main__':
    MissionPlanner()
