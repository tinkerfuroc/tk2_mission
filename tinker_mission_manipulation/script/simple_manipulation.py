#!/usr/bin/env python

import rospy
import tf
import copy
from termcolor import colored

from tinker_mission_common.all import *
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Bool
from smach import Iterator, StateMachine, Sequence, Concurrence
from smach_ros import ServiceState, MonitorState, IntrospectionServer
from tinker_vision_msgs.msg import ObjectAction
from tinker_vision_msgs.srv import FindObjects


def find_div(h, target_level = 1):
    _levels = [0.10, 0.35, 0.67, 1.06, 1.41, 1.79] 
    for i in range(0, len(_levels)):
        if i == len(_levels) - 1:
            return h - (_levels[i] - _levels[target_level])
        if _levels[i] <= h < _levels[i + 1]:
            return h - (_levels[i] - _levels[target_level])
    return 0

def main():
    rospy.init_node('tinker_mission_manipulation')
    trans = tf.TransformListener()
    rospy.loginfo("Waiting for tf ...")
    rospy.sleep(3)
    assert (len(trans.getFrameStrings()) > 0)

    state = StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
    with state:
        def kinect_callback(userdata, result):
            userdata.objects = []
            objects = []
            sum_x = 0
            for obj in result.objects.objects:
                position = obj.pose.pose.pose.position
                if position.y > 0.4 or position.y < -0.4:
                    continue
                if position.z > 1.1:
                    position.z += 0.05
                obj.header.stamp = rospy.Time(0)
                kinect_point = PointStamped(header=obj.header, point=position)
                odom_point = trans.transformPoint('odom', kinect_point)
                sum_x += odom_point.point.x
                rospy.loginfo(colored('[Kinect Object(odom)] from:(%f %f %f)', 'yellow'), odom_point.point.x,
                              odom_point.point.y, odom_point.point.z)
                objects.append(odom_point)

            avg_x = sum_x / len(objects)
            
            for from_point in objects:
                to_point = copy.deepcopy(from_point)
                to_point.point.x = avg_x 
                to_point.point.z = find_div(from_point.point.z)
                userdata.objects.append({'from': from_point, 'to': to_point})
                rospy.loginfo(colored('[Kinect Object(odom)] to:(%f %f %f)', 'yellow'), to_point.point.x,
                              to_point.point.y, to_point.point.z)
            return 'succeeded'

        StateMachine.add('Arm_Mode_Kinect', ArmModeState(ArmModeState.Arm_Mode_Kinect), 
                transitions={'succeeded': 'Start_Button'})
        StateMachine.add('Start_Button', MonitorStartButtonState(), 
                transitions={'valid': 'Start_Button', 'invalid': 'S_Kinect_Recognition'})
        StateMachine.add('S_Kinect_Recognition',
                ServiceState(service_name='/kinect_find_objects',
                    service_spec=FindObjects,
                    input_keys=['objects'],
                    output_keys=['objects'],
                    response_cb=kinect_callback),
                transitions={'succeeded': 'Generate_Report'})
        StateMachine.add('Generate_Report', GenerateReportState(image='result.png', text='object_names.txt'), 
                transitions={'succeeded': 'IT_Objects_Iterator'} )

        objects_iterator = Iterator(outcomes=['succeeded', 'preempted', 'aborted'],
                input_keys=['objects'], output_keys=[],
                it=lambda: state.userdata.objects, it_label='target',
                exhausted_outcome='succeeded')

        with objects_iterator:
            fetch_object_sequence = Sequence(outcomes=['succeeded', 'aborted', 'continue', 'preempted'],
                                             input_keys=['target'],
                                             connector_outcome='succeeded')
            with fetch_object_sequence:
                Sequence.add('Gripper_Photo', GripperState(GripperState.GRIPPER_OPEN))
                Sequence.add('Move_For_Photo', MoveArmState(Point(-0.7, 0, 0), target_key='from'))
                concurrence = Concurrence(outcomes=['succeeded', 'aborted', 'preempted'],
                                          default_outcome='succeeded',
                                          child_termination_cb=lambda x: True,
                                          input_keys=['target'])
                with concurrence:
                    Concurrence.add('Move_Fetch', MoveArmState(Point(0.1, 0, 0), target_key='from'))
                    Concurrence.add('Gripper_Laser_sensor', MonitorState('/gripper_laser_sensor', Bool, cond_cb=lambda x,y: False))

                Sequence.add('Move_Fetch_Concurrence', concurrence)
                Sequence.add('Gripper_Fetch', GripperState(GripperState.GRIPPER_CLOSE))
                Sequence.add('Move_Fetch_Back', MoveArmState(Point(-0.6, 0, 0), target_key='from'))
                Sequence.add('Move_Down', MoveArmState(Point(-0.5, 0, 0), target_key='to'))
                Sequence.add('Move_Put', MoveArmState(Point(0, 0, 0), target_key='to'))
                Sequence.add('Gripper_Put', GripperState(GripperState.GRIPPER_OPEN))
                Sequence.add('Move_Put_Back', MoveArmState(Point(-0.5, 0, 0), target_key='to'), transitions={'succeeded': 'continue'})

            Iterator.set_contained_state('Seq_Fetch_Object', fetch_object_sequence, loop_outcomes=['continue'])

        # end of objects_iterator
        StateMachine.add('IT_Objects_Iterator', objects_iterator,
                transitions= {'succeeded': 'A_Move_Reset', 'aborted': 'A_Move_Reset'})

        StateMachine.add('A_Move_Reset', ArmModeState(ArmModeState.Arm_Mode_Init),
                         transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

    # Run state machine introspection server for smach viewer
    intro_server = IntrospectionServer('tinker_mission_manipulation', state, '/tinker_mission_manipulation')
    intro_server.start()

    outcome = state.execute()
    rospy.spin()
    intro_server.stop()


if __name__ == '__main__':
    main()
