import rospy
from termcolor import colored

from smach import State 
from smach_ros import SimpleActionState
from std_msgs.msg import String
from tinker_audio_msgs.msg import RecognizeKeywordsAction, RecognizeKeywordsActionGoal

__all__ = ['SpeakState', 'KeywordsRecognizeState']


class SpeakState(State):
    pub = rospy.Publisher('/tts', String, queue_size=10)
    def __init__(self, text):
        State.__init__(self, outcomes=['succeeded'])
        self.text = text

    def execute(self, userdata):
        rospy.loginfo(colored('[Speak] %s','green'), self.text)
        SpeakState.pub.publish(self.text)
        return 'succeeded'

class KeywordsRecognizeState(SimpleActionState):
    def __init__(self, keywords='hello'):
        super(MoveArmState, self).__init__(action_name='/keywords_recognize',
                action_spec=RecognizeKeywordsAction,
                goal = RecognizeKeywordsActionGoal(keywords=keywords))

