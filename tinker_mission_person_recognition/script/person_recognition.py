#!/usr/bin/env python

import rospy
import tf
from termcolor import colored

from geometry_msgs.msg import PointStamped, Point
from smach import Iterator, StateMachine, Sequence, Concurrence
from smach_ros import ServiceState, SimpleActionState, MonitorState, IntrospectionServer

from tinker_mission_common.all import *


def main():
    rospy.init_node('tinker_mission_person_recognition')
    rospy.loginfo(colored('starting person recognition task ...', 'green'))

    # Main StateMachine
    state = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with state:
        StateMachine.add('1_Start', MonitorKinectBodyState(), transitions={'valid':'1_Start', 'invalid':'Sequence'})
        sequence = Sequence(outcomes=['succeeded', 'preempted', 'aborted'],
                            connector_outcome='succeeded')
        with sequence:
            sequence.add('2_1_train_speak', SpeakState('please look at the camera'))
            sequence.add('2_2_train', TrainPersonState())
            sequence.add('2_3_train_finish', SpeakState('I have memorized you, thanks'))
            sequence.add('3_wait', DelayState(10))
            sequence.add('4_turn_back', ChassisSimpleMoveState(theta=3.15)) 
            sequence.add('5_1_find_operator', FindPersonState()) 
            sequence.add('5_2_point_operator', MoveArmState(offset=Point(0, 0, 0)), remapping={'target':'person_pose'}) 

        StateMachine.add('Sequence', sequence, {'succeeded': 'succeeded', 'aborted': 'aborted'})

    # Run state machine introspection server for smach viewer
    intro_server = IntrospectionServer('tinker_mission_person_recognition', state, '/tinker_mission_person_recognition')
    intro_server.start()

    outcome = state.execute()
    rospy.spin()
    intro_server.stop()


if __name__ == '__main__':
    main()
