#!/usr/bin/env python


import rospy
from std_msgs.msg import Empty
from patriot_robotics.msg import PointStampedColorRGBA
from srcsim.msg import Console

class State:
    '''
    Sate machine for Qual 1.  States:
    - off      Initial state and task complete state
    - seeking  State when task starts (big white screen turns on) or tracking is lost
    - tracking State when LED is acquired
    '''
    
    def __init__(self):
        self.state = 'off'
        self.published_led = False
        rospy.Subscriber('on', Empty, self.on)
        rospy.Subscriber('off', Empty, self.off)
        rospy.Subscriber('acquired', Empty, self.acquired)
        rospy.Subscriber('lost', Empty, self.lost)
        rospy.Subscriber('led_3D_location', PointStampedColorRGBA, 
                                               self.led_3D_location)
        
        self.console_publisher = rospy.Publisher("/srcsim/qual1/light", 
                                            Console, queue_size = 20)

    def on(self, msg):
        if self.state == 'off':
            self.state= 'seeking'
            self.published_led = False
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
            self.published_led = False
            self.log_state()

    def led_3D_location(self, msg):
        if not self.published_led:
            console_msg = Console(x=msg.point_stamped.point.x,
                                  y=msg.point_stamped.point.y,
                                  z=msg.point_stamped.point.z,
                                  r=msg.color_rgba.r,
                                  g=msg.color_rgba.g,
                                  b=msg.color_rgba.g)
            self.console_publisher.publish(console_msg)
            self.published_led = True
            rospy.loginfo("Published LED")
        

    def log_state(self):
        rospy.loginfo('State is %s' % self.state)
        

if __name__ == '__main__':
    rospy.init_node('qual1_state')
    state = State()
    rospy.loginfo('Ready')
    state.log_state()
    rospy.spin()

