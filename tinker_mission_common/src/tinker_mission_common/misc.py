import rospy
from termcolor import colored

from smach import State 
from smach_ros import MonitorState, SimpleActionState
from std_msgs.msg import Bool
from tinker_vision_msgs.msg import EmptyAction, EmptyGoal

__all__ = ['MonitorStartButtonState', 'DetectDoorState', 'DelayState', 'GenerateReportState']


class MonitorStartButtonState(MonitorState):
    def __init__(self):
        super(MonitorStartButtonState, self).__init__(
                topic='/start_button',
                msg_type=Bool,
                cond_cb=lambda x,y: False)


class DetectDoorState(SimpleActionState):
    def __init__(self):
        super(DetectDoorState, self).__init__(
                action_name='/check_door', action_spec=EmptyAction,
                input_keys=[], output_keys=[],
                goal=EmptyGoal())


class DelayState(State):
    def __init__(self, time):
        State.__init__(self, outcomes=['succeeded'])
        self.time = time

    def execute(self, userdata):
        rospy.loginfo(colored('[Delay] %d','green'), self.time)
        rospy.sleep(self.time)
        return 'succeeded'


class GenerateReportState(State):
    def __init__(self, image, text):
        State.__init__(self, outcomes=['succeeded'])
        self.image = image
        self.text = text

    def execute(self, userdata):
        from fpdf import FPDF
        import os
        import time
        from os.path import expanduser

        home = expanduser("~")
        pdf = FPDF()
        pdf.set_font('Arial', 'B', 16)
        pdf.add_page()
        pdf.image(os.path.join(home, self.image), w=100)
        pdf.ln(20)
        pdf.cell(0, txt=open(os.path.join(home, self.text), 'r').read())
        pdf.output(os.path.join(home, 'Tinker-3-UTC-'+time.strftime("%H-%M-%S")+'.pdf'), 'F')
        pdf.output(os.path.join('/media/iarc/SSD/', 'Tinker-3-UTC-'+time.strftime("%H-%M-%S")+'.pdf'), 'F')
        return 'succeeded'

