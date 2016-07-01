#!/usr/bin/env python

import rospy
import tf
from termcolor import colored

from smach import StateMachine, Sequence, Concurrence
from smach_ros import  MonitorState, IntrospectionServer
from tinker_mission_common.all import *


def main():
    rospy.init_node('tinker_mission_follow')
    rospy.loginfo(colored('starting follow and guide task ...', 'green'))

    # Main StateMachine
    state = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with state:
        StateMachine.add('Start_Button', MonitorStartButtonState(), 
                transitions={'valid': 'Start_Button', 'invalid': '1_Start'})
        StateMachine.add('1_Start', MonitorKinectBodyState(), 
                transitions={'valid':'1_Start', 'invalid':'Sequence'})
        sequence = Sequence(outcomes=['succeeded', 'preempted', 'aborted'], connector_outcome='succeeded')
        with sequence:
            follow_concurrence = Concurrence(outcomes=['succeeded', 'aborted', 'preempted'],
                default_outcome='succeeded', child_termination_cb=lambda x: True, input_keys=[])

            with follow_concurrence:
                Concurrence.add('FollowMe', FollowMeState())
                Concurrence.add('KeyWordsRecognition', KeywordsRecognizeState('stop'))

            Sequence.add('Follow_concurrence', follow_concurrence)
	   
        StateMachine.add('Sequence', sequence, {'succeeded': 'succeeded', 'aborted': 'aborted'})

    # Run state machine introspection server for smach viewer
    intro_server = IntrospectionServer('tinker_mission_navigation', state, '/tinker_mission_navigation')
    intro_server.start()

    outcome = state.execute()
    rospy.spin()
    intro_server.stop()


if __name__ == '__main__':
    main()

