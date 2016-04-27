#!/usr/bin/env python

import rospy
import smach
import tf
from smach import Iterator, StateMachine, CBState
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
from tinker_object_recognition.srv import FindObjects
from tk_arm.msg import ArmReachObjectAction, ArmReachObjectGoal,ArmInitAction
from geometry_msgs.msg import PointStamped


def main():
    rospy.init_node('tk2_mission_manipulation')
    trans = tf.TransformListener()
    rospy.loginfo("Waiting for tf ...")
    rospy.sleep(3)
    assert(len(trans.getFrameStrings()) > 0)

    state = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with state:
        StateMachine.add('S_Kinect_Recognition',
                         ServiceState('/kinect_find_objects',
                                      FindObjects,
                                      response_slots=['objects']),
                         transitions={'succeeded': 'IT_Objects_Iterator'})

        objects_iterator = Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                                    input_keys=['objects'],
                                    output_keys=[],
                                    it=lambda: range(0, len(state.userdata.objects.objects)),
                                    it_label='object_index',
                                    exhausted_outcome = 'succeeded')

        with objects_iterator:
            fetch_object_state = StateMachine(
                    outcomes=['succeeded', 'preempted', 'aborted', 'continue'],
                    input_keys=['objects','object_index'])

            with fetch_object_state:
                def _move_photo_callback(userdata, goal):
                    obj = userdata.objects.objects[userdata.object_index]
                    position = obj.pose.pose.pose.position
                    obj.header.stamp = rospy.Time.now()
                    rospy.sleep(0.5)
                    kinect_point = PointStamped(header=obj.header,point=position)
                    goal_point = trans.transformPoint('arm_link',kinect_point)
                    target_x = goal_point.point.x
                    factor = (target_x - 0.4) / target_x
                    goal_point.point.x *= factor
                    goal_point.point.y *= factor
                    goal_point.point.z += 0.1
                    goal = ArmReachObjectGoal(pos=goal_point.point,grasp_state=3)
                    rospy.loginfo("[Photo] Object: %i/%i",userdata.object_index,
                            len(userdata.objects.objects))
                    rospy.loginfo("[Photo] point (%f %f %f)", goal.pos.x,goal.pos.y,goal.pos.z)
                    return goal

                StateMachine.add('A_Move_Photo',
                                 SimpleActionState('/arm_reach_position',
                                                   ArmReachObjectAction,
                                                   goal_cb=_move_photo_callback,
                                                   input_keys=['objects', 'object_index']),
                                 transitions={'succeeded': 'S_Arm_Find_Object',
                                              'aborted': 'continue'})

                StateMachine.add('S_Arm_Find_Object',
                                 ServiceState('/arm_find_objects',
                                              FindObjects),
                                 transitions={'succeeded': 'A_Move_Fetch',
                                              'aborted': 'continue'})

                def _move_fetch_callback(userdata, goal):
                    obj = userdata.objects.objects[userdata.object_index]
                    position = obj.pose.pose.pose.position
                    obj.header.stamp = rospy.Time.now()
                    rospy.sleep(0.5)
                    kinect_point = PointStamped(header=obj.header,point=position)
                    goal_point = trans.transformPoint('arm_link',kinect_point)
                    goal_point.point.x -= 0.1
                    goal_point.point.z += 0.1
                    goal = ArmReachObjectGoal(pos=goal_point.point,grasp_state=2)
                    rospy.loginfo("[Fetch] point (%f %f %f)",goal.pos.x,goal.pos.y,goal.pos.z)
                    return goal

                StateMachine.add('A_Move_Fetch',
                                 SimpleActionState('/arm_reach_position',
                                                   ArmReachObjectAction,
                                                   goal_cb=_move_fetch_callback,
                                                   input_keys=['objects', 'object_index']),
                                 transitions={'succeeded': 'A_Move_Back',
                                              'aborted': 'A_Move_Back'})

                def _move_back_callback(userdata, goal):
                    obj = userdata.objects.objects[userdata.object_index]
                    position = obj.pose.pose.pose.position
                    obj.header.stamp = rospy.Time.now()
                    rospy.sleep(0.5)
                    kinect_point = PointStamped(header=obj.header,point=position)
                    goal_point = trans.transformPoint('arm_link',kinect_point)
                    goal_point.point.x = 0.15
                    goal_point.point.y = 0.0
                    goal_point.point.z += 0.1
                    goal = ArmReachObjectGoal(pos=goal_point.point,grasp_state=2)
                    rospy.loginfo("[Back] point (%f %f %f)",goal.pos.x,goal.pos.y,goal.pos.z)
                    return goal

                StateMachine.add('A_Move_Back',
                                 SimpleActionState('/arm_reach_position',
                                                   ArmReachObjectAction,
                                                   goal_cb=_move_back_callback,
                                                   input_keys=['objects', 'object_index']),
                                 transitions={'succeeded': 'A_Move_Release',
                                              'aborted': 'A_Move_Release'})

                def _move_release_callback(userdata, goal):
                    obj = userdata.objects.objects[userdata.object_index]
                    position = obj.pose.pose.pose.position
                    kinect_point = PointStamped(header=obj.header,point=position)
                    obj.header.stamp = rospy.Time.now()
                    rospy.sleep(0.5)
                    goal_point = trans.transformPoint('arm_link',kinect_point)
                    goal_point.point.z = -0.16
                    goal = ArmReachObjectGoal(pos=goal_point.point,grasp_state=3)
                    rospy.loginfo("[Release] point (%f %f %f)",goal.pos.x,goal.pos.y,goal.pos.z)
                    return goal

                StateMachine.add('A_Move_Release',
                                 SimpleActionState('/arm_reach_position',
                                                   ArmReachObjectAction,
                                                   goal_cb=_move_release_callback,
                                                   input_keys=['objects', 'object_index']),
                                 transitions={'succeeded': 'A_Move_Release_Back',
                                              'aborted': 'A_Move_Release_Back'})
                                 
                def _move_release_back_callback(userdata, goal):
                    obj = userdata.objects.objects[userdata.object_index]
                    position = obj.pose.pose.pose.position
                    obj.header.stamp = rospy.Time.now()
                    rospy.sleep(0.5)
                    kinect_point = PointStamped(header=obj.header,point=position)
                    goal_point = trans.transformPoint('arm_link',kinect_point)
                    goal_point.point.x = 0.2
                    goal_point.point.y = 0.0
                    goal_point.point.z = -0.16
                    goal = ArmReachObjectGoal(pos=goal_point.point,grasp_state=3)
                    rospy.loginfo("[Release-Back] point (%f %f %f)",goal.pos.x,goal.pos.y,goal.pos.z)
                    return goal
                StateMachine.add('A_Move_Release_Back',
                                 SimpleActionState('/arm_reach_position',
                                                   ArmReachObjectAction,
                                                   goal_cb=_move_release_back_callback,
                                                   input_keys=['objects', 'object_index']),
                                 transitions={'succeeded': 'continue',
                                              'aborted': 'continue'})
            Iterator.set_contained_state('CN_Iterator_Container',fetch_object_state,
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
    intro_server = IntrospectionServer('tk2_mission_manipulation',state,'/tk2_mission_manipulation')
    intro_server.start()

    outcome = state.execute()
    rospy.spin()
    intro_server.stop()


if __name__ == '__main__':
    main()
