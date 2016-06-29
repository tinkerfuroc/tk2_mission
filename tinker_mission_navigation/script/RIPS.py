#!/usr/bin/env python

import rospy
import tf
from termcolor import colored

from smach import StateMachine, Sequence, Concurrence
from smach_ros import  MonitorState, IntrospectionServer
from tinker_mission_common.all import *
import tinker_knowledge.position as TKpos


def main():
    rospy.init_node('tinker_RIPS')
    rospy.loginfo(colored('starting RIPS task ...', 'green'))

    # load waypoints from xml
    pose_list = TKpos.PoseList.parse(open(rospy.get_param('~waypoint_xml'), 'r'))
    for pose in pose_list.pose:
        WayPointGoalState.waypoint_dict[pose.name] = TKpos.Pose.X(pose)

    # Main StateMachine
    state = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with state:
        StateMachine.add('Start_Button', MonitorStartButtonState(), 
                transitions={'valid': 'Start_Button', 'invalid': 'Sequence'})

        sequence_1 = Sequence(outcomes=['succeeded', 'preempted', 'aborted'],
                connector_outcome='succeeded')
        with sequence_1:
            Sequence.add('GoToWaypoin1', WayPointGoalState('waypoint1'), transitions={'aborted': 'GoToWaypoin1'})
            Sequence.add('ArriveWaypoint1', SpeakState('Wait for continue'))

        StateMachine.add('Sequence', sequence_1, {'succeeded': 'Continue', 'aborted': 'aborted'})
        StateMachine.add('Continue', MonitorStartButtonState(), 
                transitions={'valid': 'Continue', 'invalid': 'Sequence_2'})
        sequence_2 = Sequence(outcomes=['succeeded', 'preempted', 'aborted'],
                connector_outcome='succeeded')
        with sequence_2:
            Sequence.add('GoToWaypoin2', WayPointGoalState('waypoint2'), transitions={'aborted': 'GoToWaypoin2'}) 
            Sequence.add('ArriveWaypoint2', SpeakState('I have arrived at way point two'))
            Sequence.add('GoToWaypoin3', WayPointGoalState('waypoint3'), transitions={'aborted': 'GoToWaypoin3'})
            Sequence.add('ArriveWaypoint3', SpeakState('I have arrived at way point three'))
        StateMachine.add('Sequence_2', sequence_2, {'succeeded': 'succeeded', 'aborted': 'aborted'})

    # Run state machine introspection server for smach viewer
    intro_server = IntrospectionServer('tinker_mission_navigation', state, '/tinker_mission_navigation')
    intro_server.start()

    outcome = state.execute()
    rospy.spin()
    intro_server.stop()


if __name__ == '__main__':
    main()

