import rospy

from geometry_msgs.msg import Pose2D
from smach_ros import SimpleActionState
from tinker_msgs.msg import SimpleMoveAction, SimpleMoveGoal

__all__ = ['ChassisSimpleMoveState']


class ChassisSimpleMoveState(SimpleActionState):
    def __init__(self, x=0, y=0, theta=0):
        super(ChassisSimpleMoveState, self).__init__(action_name='/simple_move',
                action_spec=SimpleMoveAction,
                goal = SimpleMoveGoal(target=Pose2D(x=x, y=y, theta=theta)))

