import rospy
from termcolor import colored

from smach_ros import SimpleActionState
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tinker_msgs.msg import SimpleMoveAction, SimpleMoveGoal, FollowAction

__all__ = ['ChassisSimpleMoveState', 'WayPointGoalState', 'FollowMeState']


class ChassisSimpleMoveState(SimpleActionState):
    def __init__(self, x=0, y=0, theta=0):
        super(ChassisSimpleMoveState, self).__init__(action_name='/simple_move',
                action_spec=SimpleMoveAction,
                goal = SimpleMoveGoal(target=Pose2D(x=x, y=y, theta=theta)))


class WayPointGoalState(SimpleActionState):
    waypoint_dict = {}
    def __init__(self, waypoint):
        super(WayPointGoalState, self).__init__(
                action_name='/move_base', action_spec=MoveBaseAction,
                input_keys=['waypoint'], output_keys=[],
                goal_cb=self.goal_callback, goal_cb_args=(waypoint,))

    @staticmethod
    def goal_callback(userdata, goal, waypoint):
        rospy.loginfo(colored('[Nav] %s', 'green'), waypoint)
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = WayPointGoalState.waypoint_dict[waypoint]
        goal = MoveBaseGoal(target_pose=pose_stamped)
        return goal


class FollowMeState(SimpleActionState):
    def __init__(self):
        super(FollowMeState, self).__init__(
                action_name='/follow_action', action_spec=FollowAction,
                input_keys=[], output_keys=[])

