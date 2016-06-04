#!/usr/bin/env python

import rospy
import tf
import copy

from numpy import interp
from termcolor import colored

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Bool
from smach import Iterator, StateMachine, Sequence, Concurrence
from smach_ros import ServiceState, SimpleActionState, MonitorState, IntrospectionServer
from tinker_vision_msgs.msg import ObjectAction
from tinker_vision_msgs.srv import FindObjects
from tk_arm.msg import ArmReachObjectAction, ArmReachObjectGoal, ArmInitAction, ArmHandAction, ArmHandGoal

GRIPPER_OPEN = False
GRIPPER_CLOSE = True

K_Image_x = (-300, -106, 10.5, 128, 300)
K_Arm_y = (0.15, 0.1, 0, -0.1, -0.15)
K_Image_y = (-300, -170, 170, 300)
K_Arm_z = (0.20, 0.10, 0., -0.10)


class MoveArmState(SimpleActionState):
    def __init__(self, offset=Point(x=0, y=0, z=0), **kwargs):
        super(MoveArmState, self).__init__(action_name='/tinker_arm_move',
                                           action_spec=ArmReachObjectAction,
                                           input_keys=['objects', 'object_index'],
                                           output_keys=[],
                                           goal_cb=self.goal_callback,
                                           goal_cb_args=(offset,),
                                           goal_cb_kwargs=kwargs)

    @staticmethod
    def goal_callback(userdata, goal, offset, **kwargs):
        object_pos = userdata.objects[userdata.object_index]
        goal_point = copy.deepcopy(object_pos)
        goal_point.point.x += offset.x
        goal_point.point.y += offset.y
        goal_point.point.z += offset.z
        if kwargs.has_key('abs_z'):
            goal_point.point.z = kwargs['abs_z']
        point = goal_point.point
        rospy.loginfo(colored("Object [%d/%d]: (%f, %f, %f)", 'green'), userdata.object_index + 1,
                      len(userdata.objects),
                      point.x, point.y, point.z)
        goal = ArmReachObjectGoal(pos=goal_point, state=0)
        return goal


class GripperState(SimpleActionState):
    def __init__(self, gripper_state=GRIPPER_CLOSE):
        super(GripperState, self).__init__(action_name='/arm_hand',
                                           action_spec=ArmHandAction,
                                           goal=ArmHandGoal(state=gripper_state))


def image_compensate(userdata, status, result):
    if status == GoalStatus.SUCCEEDED:
        object_pos = userdata.objects[userdata.object_index].point
        image_pixel = result.objects.objects[0].pose.pose.pose.position
        object_pos.y += interp(image_pixel.x, K_Image_x, K_Arm_y)
        object_pos.z += interp(image_pixel.y, K_Image_y, K_Arm_z)
        rospy.loginfo(colored('[Image Compensate]', 'green'))
        rospy.loginfo(colored('[Image pixel] (x=%f y=%f)', 'yellow'), image_pixel.x, image_pixel.y)
        rospy.loginfo(colored('[Object(compensated)] (%f %f %f)', 'yellow'), object_pos.x, object_pos.y, object_pos.z)
        return 'succeeded'


def main():
    rospy.init_node('tinker_mission_manipulation')
    trans = tf.TransformListener()
    rospy.loginfo("Waiting for tf ...")
    rospy.sleep(3)
    assert (len(trans.getFrameStrings()) > 0)

    state = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with state:
        def kinect_callback(userdata, result):
            objects_raw = result.objects
            userdata.objects = []
            for obj in objects_raw.objects:
                position = obj.pose.pose.pose.position
                if position.y > 0.4 or position.y < -0.4:
                    continue
                if position.z > 1.1:
                    position.z += 0.05
                obj.header.stamp = rospy.Time(0)
                kinect_point = PointStamped(header=obj.header, point=position)
                odom_point = trans.transformPoint('odom', kinect_point)
                rospy.loginfo(colored('[Kinect Object(odom)] (%f %f %f)', 'yellow'), odom_point.point.x,
                              odom_point.point.y, odom_point.point.z)
                userdata.objects.append(odom_point)
            return 'succeeded'

        StateMachine.add('S_Kinect_Recognition',
                         ServiceState(service_name='/kinect_find_objects',
                                      service_spec=FindObjects,
                                      input_keys=['objects'],
                                      output_keys=['objects'],
                                      response_cb=kinect_callback),
                         transitions={'succeeded': 'IT_Objects_Iterator'})

        objects_iterator = Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                                    input_keys=['objects'],
                                    output_keys=[],
                                    it=lambda: range(0, len(state.userdata.objects)),
                                    it_label='object_index',
                                    exhausted_outcome='succeeded')

        with objects_iterator:
            fetch_object_sequence = Sequence(outcomes=['succeeded', 'aborted', 'continue', 'preempted'],
                                             input_keys=['objects', 'object_index'],
                                             connector_outcome='succeeded')
            with fetch_object_sequence:
                Sequence.add('Gripper_Photo', GripperState(GRIPPER_OPEN))
                Sequence.add('Move_For_Photo', MoveArmState(Point(-0.7, 0, 0)),
                             transitions={'aborted': 'continue'})
                concurrence = Concurrence(outcomes=['succeeded', 'aborted', 'preempted'],
                                          default_outcome='succeeded',
                                          child_termination_cb=lambda x: True,
                                          input_keys=['objects', 'object_index'])
                with concurrence:
                    Concurrence.add('Move_Fetch', MoveArmState(Point(0.1, 0, 0)))
                    Concurrence.add('Gripper_Laser_sensor',
                                    MonitorState('/gripper_laser_sensor',
                                                 Bool,
                                                 cond_cb=lambda x,y: False))
                Sequence.add('Move_Fetch_Concurrence', concurrence)
                Sequence.add('Gripper_Fetch', GripperState(GRIPPER_CLOSE))
                Sequence.add('Move_Fetch_Back', MoveArmState(Point(-1, 0, 0)))
                Sequence.add('Move_Down', MoveArmState(Point(-1, 0, 0), abs_z=0.65))
                Sequence.add('Move_Put', MoveArmState(Point(-0.7, 0, 0), abs_z=0.65))
                Sequence.add('Gripper_Put', GripperState(GRIPPER_OPEN))
                Sequence.add('Move_Put_Back', MoveArmState(Point(-1, 0, 0), abs_z=0.65),
                             transitions={'succeeded': 'continue'})

            Iterator.set_contained_state('Seq_Fetch_Object', fetch_object_sequence,
                                         loop_outcomes=['continue'])

        # end of objects_iterator
        StateMachine.add('IT_Objects_Iterator',
                         objects_iterator,
                         {'succeeded': 'A_Move_Reset',
                          'aborted': 'A_Move_Reset'})

        StateMachine.add('A_Move_Reset',
                         SimpleActionState('/arm_reset',
                                           ArmInitAction,
                                           input_keys=[]),
                         transitions={'succeeded': 'succeeded',
                                      'aborted': 'aborted'})
    # Run state machine introspection server for smach viewer
    intro_server = IntrospectionServer('tinker_mission_manipulation', state, '/tinker_mission_manipulation')
    intro_server.start()

    outcome = state.execute()
    rospy.spin()
    intro_server.stop()


if __name__ == '__main__':
    main()
