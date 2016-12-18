#!/usr/bin/env python


import rospy
from std_msgs.msg import Empty


class State:
    '''
    Sate machine for Qual 1.  States:
    - off      Initial state and task complete state
    - seeking  State when task starts (big white screen turns on) or tracking is lost
    - tracking State when LED is acquired
    '''
    
    def __init__(self):
        self.state = 'off'
        self.on_subscriber = rospy.Subscriber('on', Empty, self.on)
        self.off_subscriber = rospy.Subscriber('off', Empty, self.off)
        self.off_subscriber = rospy.Subscriber('acquired', Empty, self.acquired)
        self.off_subscriber = rospy.Subscriber('lost', Empty, self.lost)

        
    def on(self, msg):
        if self.state == 'off':
            self.state= 'seeking'
            self.log_state()
        
    def off(self, msg):
        if self.state == 'seeking':
            self.state= 'off'
            self.log_state()

    def acquired(self, msg):
        if self.state == 'seeking':
            self.state= 'tracking'
            self.log_state()

    def lost(self, msg):
        if self.state == 'tracking':
            self.state= 'seeking'
            self.log_state()

    def log_state(self):
        rospy.loginfo('State is %s' % self.state)
        

if __name__ == '__main__':
    rospy.init_node('qual1_state')
    state = State()
    rospy.loginfo('Ready')
    state.log_state()
    rospy.spin()

