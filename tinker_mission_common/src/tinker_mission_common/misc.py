import rospy
from termcolor import colored

from smach import State 

__all__ = ['DelayState']


class DelayState(State):
    def __init__(self, time):
        State.__init__(self, outcomes=['succeeded'])
        self.time = time

    def execute(self, userdata):
        rospy.loginfo(colored('[Delay] %d','green'), self.time)
        rospy.sleep(self.time)
        return 'succeeded'
