#!/usr/bin/env python

import rospy
import tf
from termcolor import colored

from tinker_mission_common.all import *
from smach import Iterator, StateMachine, Sequence, Concurrence
from smach_ros import ServiceState, SimpleActionState, MonitorState, IntrospectionServer
from geometry_msgs.msg import PointStamped, Point, Pose2D


def main():
    rospy.init_node('tinker_mission_person_recognition')
    rospy.loginfo(colored('starting person recognition task ...', 'green'))

    # Main StateMachine
    state = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with state:
        StateMachine.add('0_wait_for_start', MonitorStartButtonState(), transitions={'valid': '0_wait_for_start','invalid': '0_arm_mode'})
        StateMachine.add('0_arm_mode', ArmModeState(ArmModeState.Arm_Mode_Kinect), transitions={'succeeded':'0_wait_for_human'})
        StateMachine.add('0_wait_for_human', SpeakState('I am tinker , I am waiting for operator'), 
                transitions={'succeeded': '1_Start'})
        StateMachine.add('1_Start', MonitorKinectBodyState(), transitions={'valid':'1_Start', 'invalid':'Sequence'})
        sequence = Sequence(outcomes=['succeeded', 'preempted', 'aborted'],
                            connector_outcome='succeeded')
        with sequence:
            Sequence.add('2_1_train_speak', SpeakState('please look at the camera'))
            Sequence.add('2_2_train', TrainPersonState())
            Sequence.add('2_3_train_finish', SpeakState('I have memorized you, thanks'))
            Sequence.add('3_wait', DelayState(10))
            Sequence.add('4_turn_back', ChassisSimpleMoveState(theta=3.23)) 
            Sequence.add('4_1_turn_back', ChassisSimpleMoveState(x=-1.0)) 
            Sequence.add('5_1_find_operator', FindPersonState()) 
            Sequence.add('5_2_point_operator', MoveArmState(offset=Point(0, 0, 0)), remapping={'target':'person_pose'}) 
            Sequence.add('5_3_say', SpeakState('I have found you'))
            Sequence.add('5_4_wait', DelayState(5))

        StateMachine.add('Sequence', sequence, {'succeeded': 'Sequence_reset', 'aborted': 'Sequence_reset'})

        sequence_reset = Sequence(outcomes=['succeeded', 'preempted', 'aborted'],
                            connector_outcome='succeeded')
        with sequence_reset:
            Sequence.add('6_1_arm_init', ArmModeState(ArmModeState.Arm_Mode_Init))
            Sequence.add('6_2_report', GenerateReportState(image='human_result.png', text='human_result.txt'))
            Sequence.add('6_3_finish', SpeakState('task finished'))

        StateMachine.add('Sequence_reset', sequence_reset, {'succeeded': 'succeeded', 'aborted': 'aborted'})

    # Run state machine introspection server for smach viewer
    intro_server = IntrospectionServer('tinker_mission_person_recognition', state, '/tinker_mission_person_recognition')
    intro_server.start()

    outcome = state.execute()
    rospy.spin()
    intro_server.stop()


if __name__ == '__main__':
    main()
