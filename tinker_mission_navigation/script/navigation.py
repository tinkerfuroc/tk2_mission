#!/usr/bin/env python

import rospy
import tf
from termcolor import colored

from smach import StateMachine, Sequence, Concurrence
from smach_ros import  MonitorState, IntrospectionServer
from tinker_mission_common.all import *
import tinker_knowledge.position as TKpos


def main():
    rospy.init_node('tinker_mission_navigation')
    rospy.loginfo(colored('starting navigation task ...', 'green'))

    # load waypoints from xml
    pose_list = TKpos.PoseList.parse(open(rospy.get_param('~waypoint_xml'), 'r'))
    for pose in pose_list.pose:
        WayPointGoalState.waypoint_dict[pose.name] = TKpos.Pose.X(pose)

    # Main StateMachine
    state = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with state:
        StateMachine.add('Start_Button', MonitorStartButtonState(), 
                transitions={'valid': 'Start_Button', 'invalid': 'Sequence'})

        sequence = Sequence(outcomes=['succeeded', 'preempted', 'aborted'],
                connector_outcome='succeeded')
        with sequence:
#            Sequence.add('GoToWaypoin1', WayPointGoalState('waypoint1'), transitions={'aborted': 'GoToWaypoin1'})
#            Sequence.add('ArriveWaypoint1', SpeakState('I have arrived at way point one'))
#            Sequence.add('GoToWaypoin2', WayPointGoalState('waypoint2'), 
#                    transitions={'succeeded': 'ArriveWaypoint2', 'aborted': 'Obstacle'}) 
#            Sequence.add('Obstacle', SpeakState('Obstacle in front of me'))
#            Sequence.add('ObstacleDelay', DelayState(10),
#                    transitions={'succeeded': 'GoToWaypoin2'}) 
#            Sequence.add('ArriveWaypoint2', SpeakState('I have arrived at way point two'))
            Sequence.add('GoToWaypoin3', WayPointGoalState('waypoint3'), transitions={'aborted': 'GoToWaypoin3'})
            Sequence.add('ArriveWaypoint3', SpeakState('I have arrived at way point three'))
            Sequence.add('StopCommandAndGo', SpeakState('Please GO. If you want to stop, say stop tinker'))
            Sequence.add('Train_human', FollowTrainState())

            follow_concurrence = Concurrence(outcomes=['succeeded', 'aborted', 'preempted'],
                default_outcome='succeeded', child_termination_cb=lambda x: True, input_keys=[])
            with follow_concurrence:
                Concurrence.add('FollowMe', FollowMeState())
#                Concurrence.add('KeyWordsRecognition', KeywordsRecognizeState('stop'))

            Sequence.add('Follow_concurrence', follow_concurrence)
            Sequence.add('GoToWaypoin3Again', WayPointGoalState('waypoint3'), transitions={'aborted': 'GoToWaypoin3Again'})
            Sequence.add('ArriveWaypoint3Again', SpeakState('I am back at way point three'))
            Sequence.add('GoOut', WayPointGoalState('out'), transitions={'aborted': 'GoOut'})
	   
        StateMachine.add('Sequence', sequence, {'succeeded': 'succeeded', 'aborted': 'aborted'})

    # Run state machine introspection server for smach viewer
    intro_server = IntrospectionServer('tinker_mission_navigation', state, '/tinker_mission_navigation')
    intro_server.start()

    outcome = state.execute()
    rospy.spin()
    intro_server.stop()


if __name__ == '__main__':
    main()

