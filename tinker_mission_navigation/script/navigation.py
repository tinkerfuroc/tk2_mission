#!/usr/bin/env python

import rospy
import tf
import copy
from termcolor import colored

from smach import State, Iterator, StateMachine, Sequence, Concurrence
from smach_ros import ServiceState, SimpleActionState, MonitorState, IntrospectionServer

import tinker_knowledge.position as TKpos
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from tinker_mission_common.all import *


class WayPointGoalState(SimpleActionState):
    waypoint_dict = {}
    def __init__(self, waypoint):
        super(WayPointGoalState, self).__init__(action_name='/move_base',
                                                action_spec=MoveBaseAction,
                                                input_keys=['waypoints'],
                                                output_keys=[],
                                                goal_cb=self.goal_callback,
                                                goal_cb_args=(waypoint,))

    @staticmethod
    def goal_callback(userdata, goal, waypoint):
        rospy.loginfo(colored('[Nav] %s', 'green'), waypoint)
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = WayPointGoalState.waypoint_dict[waypoint]
        goal = MoveBaseGoal(target_pose=pose_stamped)
        return goal

class FollowMe():
	#FIXME



def main():
    rospy.init_node('tinker_mission_navigation')
    rospy.loginfo(colored('starting navigation task ...', 'green'))

    # load waypoints from xml
    pose_list = TKpos.PoseList.parse(open(rospy.get_param('~waypoint_xml', 'r')))
    for pose in pose_list.pose:
        WayPointGoalState.waypoint_dict[pose.name] = TKpos.Pose.X(pose)

    raw_input('Wait for start')

    # Main StateMachine
    state = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with state:
        sequence = Sequence(outcomes=['succeeded', 'preempted', 'aborted'],
                connector_outcome='succeeded')
        with sequence:
            Sequence.add('GoToWaypoin1', WayPointGoalState('waypoint1'))
            Sequence.add('ArriveWaypoint1', SpeakState('I have arrived at way point one'))

            Sequence.add('GoToWaypoin2', WayPointGoalState('waypoint2'))
            #FIXME
            #aborted to itself
            Sequence.add('ArriveWaypoint2', SpeakState('I have arrived at way point two'))

            Sequence.add('GoToWaypoin3', WayPointGoalState('waypoint2'))
            Sequence.add('ArriveWaypoint3', SpeakState('I have arrived at way point three'))

            Sequence.add('FindWalkers', FollowMe()) #publish walkers pose
            #FIXME
            Sequence.add('StopCommandAndGo', SpeakState('Please GO. If you want to stop, say stop tinker'))
            Sequence.add('KeyWordsRecognition', KeywordsRecognizeState('stop tinker'))

            Sequence.add('GoToWaypoin3Again', WayPointGoalState('waypoint3'))
            Sequence.add('ArriveWaypoint3Again', SpeakState('I have arrived at way point three'))
            Sequence.add('GoOut', WayPointGoalState('waypoint0'))
	   

        StateMachine.add('Sequence', sequence, {'succeeded': 'succeeded', 'aborted': 'aborted'})

    # Run state machine introspection server for smach viewer
    intro_server = IntrospectionServer('tinker_mission_navigation', state, '/tinker_mission_navigation')
    intro_server.start()

    outcome = state.execute()
    rospy.spin()
    intro_server.stop()


if __name__ == '__main__':
    main()
