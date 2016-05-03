#!/usr/bin/env python

import copy

import rospy
import tf
from geometry_msgs.msg import PointStamped, Point
from numpy import interp
from smach import Iterator, StateMachine, Sequence
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
from termcolor import colored
from tinker_vision_msgs.srv import FindObjects
from tk_arm.msg import ArmReachObjectAction, ArmReachObjectGoal, ArmInitAction

GRASP_OPEN = 2
GRASP_CLOSE = 3

K_Image_x = (-300, -106, 10.5, 128, 300)
K_Arm_y = (0.15, 0.1, 0, -0.1, -0.15)
K_Image_y = (-300, -170, 170, 300)
K_Arm_z = (0.15, 0.05, -0.05, -0.15)


class MoveArmState(SimpleActionState):
    def __init__(self, offset=Point(x=0, y=0, z=0), grasp_state=GRASP_OPEN, **kwargs):
        super(MoveArmState, self).__init__(action_name='/arm_planner',
                                           action_spec=ArmReachObjectAction,
                                           input_keys=['objects', 'object_index'],
                                           output_keys=[],
                                           goal_cb=self.goal_callback,
                                           goal_cb_args=(offset, grasp_state),
                                           goal_cb_kwargs=kwargs)

    def goal_callback(self, userdata, goal, offset, grasp_state, **kwargs):
        object_pos = userdata.objects[userdata.object_index]
        goal_point = copy.deepcopy(object_pos)
        goal_point.point.x += offset.x
        goal_point.point.y += offset.y
        goal_point.point.z += offset.z + 0.05
        if kwargs.has_key('abs_z'):
            goal_point.point.z = kwargs['abs_z']
        point = goal_point.point
        rospy.loginfo(colored("Object [%d/%d]: (%f, %f, %f)", 'green'), userdata.object_index+1, len(userdata.objects),
                point.x, point.y, point.z)
        goal = ArmReachObjectGoal(pos=goal_point, state=grasp_state)
        return goal


def image_compensate(userdata, result):
    object_pos = userdata.objects[userdata.object_index].point
    image_pixel = result.objects.objects[0].pose.pose.pose.position
    object_pos.y += interp(image_pixel.x, K_Image_x, K_Arm_y)
    object_pos.z += interp(image_pixel.y, K_Image_y, K_Arm_z)
    rospy.loginfo(colored('[Image Compensate]','green'))
    rospy.loginfo(colored('[Image pixel] (x=%f y=%f)','yellow'),image_pixel.x,image_pixel.y)
    rospy.loginfo(colored('[Object(compensated)] (%f %f %f)','yellow'),object_pos.x,object_pos.y,object_pos.z)
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
                Sequence.add('Move_For_Photo', MoveArmState(Point(-0.6, 0, 0), GRASP_OPEN))
                Sequence.add('Arm_Find_Object',
                             ServiceState(service_name='/arm_find_objects',
                                          service_spec=FindObjects,
                                          input_keys=['objects', 'object_index'],
                                          output_keys=['objects'],
                                          response_cb=image_compensate),
                             transitions={'aborted': 'continue'})
                Sequence.add('Move_Fetch', MoveArmState(Point(-0.2, 0, 0), GRASP_CLOSE))
                Sequence.add('Move_Fetch_Back', MoveArmState(Point(-1, 0, 0), GRASP_CLOSE))
                Sequence.add('Move_Down', MoveArmState(Point(-1, 0, 0), GRASP_CLOSE, abs_z=0.6))
                Sequence.add('Move_Put', MoveArmState(Point(-0.18, 0, 0), GRASP_OPEN, abs_z=0.6))
                Sequence.add('Move_Put_Back', MoveArmState(Point(-1, 0, 0), GRASP_OPEN, abs_z=0.6),
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
