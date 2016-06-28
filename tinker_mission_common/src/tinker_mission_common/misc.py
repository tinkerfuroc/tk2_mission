import rospy
from termcolor import colored

from smach import State 
from smach_ros import MonitorState
from std_msgs.msg import Bool

__all__ = ['MonitorStartButtonState', 'DelayState', 'GenerateReportState']


class MonitorStartButtonState(MonitorState):
    def __init__(self):
        super(MonitorStartButtonState, self).__init__(
                topic='/start_button',
                msg_type=Bool,
                cond_cb=lambda x,y: False)


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
        pdf.cell(0, txt=self.text)
        pdf.output(os.path.join(home, 'Tinker-'+time.strftime("%H-%M-%S")+'.pdf'), 'F')
        return 'succeeded'

