#!/usr/bin/python

import sys
import rospy
from tinker_vision_msgs.msg import EmptyGoal
from tinker_vision_msgs.msg import EmptyAction
from tinker_vision_msgs.msg import EmptyResult
from actionlib_msgs.msg import GoalStatus
import tf
from math import acos, sqrt, cos, sin
from threading import Lock
import numpy as np
import actionlib
from sensor_msgs.msg import LaserScan

class DoorChecker:
    def __init__(self):
        self.check_server = rospy.actionlib.SimpleActionServer('check_door',
                            EmptyAction,
                            auto_start = False)
        self.check_server.register_goal_callback(self.goal_cb)
        self.check_server.register_preempt_callback(self.preempt_cb)
        self.laser_sub = rospy.Subscriber('/laser', PointStamped, 
                self.found_people_handler)
        self.is_checking = False
        self.check_server.start()

    def found_people_handler(self, laser_data):
        if not self.is_checking:
            return
        mid = int(len(laser_data.ranges) / 2)
        avg_depth = np.mean(laser_data.ranges[mid - 10, mid + 10])
        if avg_depth < 0.3:
            self.follow_server.set_succeeded(EmptyResult())
            self.is_checking = False

    def goal_cb(self):
        self.follow_server.acceptNewGoal()
        self.is_checking = True

    def preempt_cb(self):
        self.follow_server.set_preempted()
        self.is_checking = False


def init(argv):
    global pub, pose, l
    rospy.init_node('simple_follow')
    follow_action = Follower()
    rospy.spin()


if __name__ == '__main__':
    init(sys.argv)

