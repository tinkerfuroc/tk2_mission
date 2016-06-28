import rospy
import copy
from termcolor import colored

from geometry_msgs.msg import PointStamped, Point
from smach_ros import SimpleActionState
from tk_arm.msg import ArmReachObjectAction, ArmReachObjectGoal, ArmInitAction, ArmInitGoal, ArmHandAction, ArmHandGoal

__all__ = ['MoveArmState', 'ArmModeState', 'GripperState']


class MoveArmState(SimpleActionState):
    def __init__(self, offset=Point(x=0, y=0, z=0), **kwargs):
        super(MoveArmState, self).__init__(action_name='/tinker_arm_move',
                                           action_spec=ArmReachObjectAction,
                                           input_keys=['target'],
                                           output_keys=[],
                                           goal_cb=self.goal_callback,
                                           goal_cb_args=(offset,),
                                           goal_cb_kwargs=kwargs)

    @staticmethod
    def goal_callback(userdata, goal, offset, **kwargs):
        object_pos = userdata.target
        goal_point = copy.deepcopy(object_pos)
        goal_point.point.x += offset.x
        goal_point.point.y += offset.y
        goal_point.point.z += offset.z
        if kwargs.has_key('abs_z'):
            goal_point.point.z = kwargs['abs_z']
        point = goal_point.point
        rospy.loginfo(colored("[Arm] (%f, %f, %f)", 'green'), point.x, point.y, point.z)
        goal = ArmReachObjectGoal(pos=goal_point, state=0)
        return goal



class ArmModeState(SimpleActionState):
    Arm_Mode_Init = 0
    Arm_Mode_Ready = 1
    Arm_Mode_Kinect = 2

    def __init__(self, mode=Arm_Mode_Init):
        super(ArmModeState, self).__init__(action_name='/arm_reset',
                action_spec=ArmInitAction,
                goal=ArmInitGoal(state=mode))
                


class GripperState(SimpleActionState):
    GRIPPER_OPEN = False
    GRIPPER_CLOSE = True

    def __init__(self, gripper_state=GRIPPER_CLOSE):
        super(GripperState, self).__init__(action_name='/arm_hand',
                                           action_spec=ArmHandAction,
                                           goal=ArmHandGoal(state=gripper_state))

