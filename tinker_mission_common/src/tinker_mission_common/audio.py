import rospy
from termcolor import colored

from smach import State 
from smach_ros import SimpleActionState
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from tinker_audio_msgs.msg import RecognizeKeywordsAction, RecognizeKeywordsGoal

__all__ = ['SpeakState', 'KeywordsRecognizeState']


class SpeakState(State):
    pub = rospy.Publisher('/tts', String, queue_size=10)
    def __init__(self, text):
        State.__init__(self, outcomes=['succeeded'])
        self.text = text

    def execute(self, userdata):
        rospy.loginfo(colored('[Speak] %s','green'), self.text)
        SpeakState.pub.publish(self.text)
        rospy.sleep(5)
        return 'succeeded'

class KeywordsRecognizeState(SimpleActionState):
    def __init__(self, keyword='hello'):
        super(KeywordsRecognizeState, self).__init__(action_name='/keywords_recognize',
                action_spec=RecognizeKeywordsAction,
                goal = RecognizeKeywordsGoal(keyword=keyword),
                result_cb=KeywordsRecognizeState.result_callback)

    @staticmethod
    def result_callback(userdata, status, result):
        if status == GoalStatus.SUCCEEDED:
            return 'succeeded'
        else:
            return 'aborted'
