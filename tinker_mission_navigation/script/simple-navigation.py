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
            Sequence.add('DetectDoor', DetectDoorState(), transitions={'aborted': 'DetectDoor'})
            Sequence.add('SimpleMove', ChassisSimpleMoveState(x=1.5))

            Sequence.add('GoToWaypoin2', WayPointGoalState('waypoint2'), 
                    transitions={'succeeded': 'ArriveWaypoint2', 'aborted': 'Obstacle'}) 
            Sequence.add('Obstacle', SpeakState('Obstacle in front of me Please remove it'))
            Sequence.add('ObstacleDelay', DelayState(10),
                    transitions={'succeeded': 'GoToWaypoin2'}) 
            Sequence.add('ArriveWaypoint2', SpeakState('I have arrived at way point two'))

            Sequence.add('GoToWaypoin2_1', WayPointGoalState('waypoint2_1'), transitions={'aborted': 'GoToWaypoin2_1'})
            Sequence.add('GoToWaypoin2_2', WayPointGoalState('waypoint2_2'), transitions={'aborted': 'GoToWaypoin2_2'})
            Sequence.add('GoToWaypoin1', WayPointGoalState('waypoint1'), transitions={'aborted': 'GoToWaypoin1'})
            Sequence.add('ArriveWaypoint1', SpeakState('I have arrived at way point one'))
            Sequence.add('GoToWaypoin1_1', WayPointGoalState('waypoint1_1'), transitions={'aborted': 'GoToWaypoin1_1'})

            Sequence.add('GoToWaypoin3', WayPointGoalState('waypoint3'), transitions={'aborted': 'GoToWaypoin3'})
            Sequence.add('ArriveWaypoint3', SpeakState('I have arrived at way point three'))
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

