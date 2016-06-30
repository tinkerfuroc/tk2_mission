import rospy

from smach_ros import ServiceState, MonitorState
from tinker_vision_msgs.srv import FindOperator
from k2_client.msg import BodyArray
from std_srvs.srv import Trigger

__all__ = ['MonitorKinectBodyState', 'TrainPersonState', 'FindPersonState']


class MonitorKinectBodyState(MonitorState):
    def __init__(self):
        super(MonitorKinectBodyState, self).__init__(
                topic='/head/kinect2/bodyArray',
                msg_type=BodyArray,
                cond_cb=lambda x,y: False)


class TrainPersonState(ServiceState):
    def __init__(self):
        super(TrainPersonState, self).__init__(
                service_name='/train_operator',
                service_spec=Trigger,
                input_keys=[],
                output_keys=[])
        

class FindPersonState(ServiceState):
    def __init__(self):
        super(FindPersonState, self).__init__(
                service_name='/find_operator',
                service_spec=FindOperator,
                input_keys=[],
                output_keys=['person_pose'],
                response_cb=FindPersonState.find_person_callback)

    @staticmethod
    def find_person_callback(userdata, result):
        if result is not None:
            result.loc.point.x -= 1.2
            userdata.person_pose = result.loc
            return 'succeeded'
        else:
            return 'aborted'

