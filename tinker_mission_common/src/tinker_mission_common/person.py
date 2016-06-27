import rospy

from smach_ros import ServiceState
from tinker_vision_msgs.srv import FindOperator
from std_srvs.srv import Empty

__all__ = ['TrainPersonState', 'FindPersonState']


class TrainPersonState(ServiceState):
    def __init__(self):
        super(ServiceState, self).__init__(
                service_name='/train_operator',
                service_spec=Empty,
                input_keys=[],
                output_keys=[])
        

class FindPersonState(ServiceState):
    def __init__(self):
        super(ServiceState, self).__init__(
                service_name='/find_operator',
                service_spec=FindOperator,
                input_keys=[],
                output_keys=['person_pose'],
                response_cb=find_person_callback)

    @staticmethod
    def find_person_callback(userdata, result):
        result.pose.position.z -= 0.1
        userdata.person_pose = result
        return 'succeeded'

