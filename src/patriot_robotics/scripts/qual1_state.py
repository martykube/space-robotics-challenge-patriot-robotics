#!/usr/bin/env python


import rospy
from std_msgs.msg import Empty
from patriot_robotics.msg import PointStampedColorRGBA
from srcsim.msg import Console

class State:
    '''
    Sate machine for Qual 1.  States:
    - seeking  Looking for LED  Message to acquired transitions state to tracking
    - tracking Found a LED  Message to lost transitions state to seeking
    '''
    
    def __init__(self):
        rospy.Subscriber('acquired', Empty, self.acquired)
        rospy.Subscriber('lost', Empty, self.lost)
        rospy.Subscriber('led_3D_location', PointStampedColorRGBA, 
                                               self.led_3D_location)
        
        self.console_publisher = rospy.Publisher("/srcsim/qual1/light", 
                                            Console, queue_size = 20)
        self.seeking = 'seeking'
        self.tracking = 'tracking'
        self.state = self.seeking
        self.acquired_led = False
        self.published_led = False
        self.acquired_count = 0
        self.max_acquired_count = 5

    def acquired(self, msg):
        if self.state == self.seeking:
            self.state= self.tracking
            self.acquired_led = True
            self.published_led = False
            self.acquired_count = 0
            self.log_state()
        if self.state == self.tracking:
            self.acquired_count += 1
            if self.acquired_count > self.max_acquired_count and not self.published_led:
                self.publish_fallback()
                

    def lost(self, msg):
        if self.state == self.tracking:
            self.state = self.seeking
            # first time acquired_led should be false
            if self.acquired_led and not self.published_led:
                self.publish_fallback()
            self.acquired_led = False
            self.published_led = False
            self.log_state()

    def led_3D_location(self, msg):

        if self.published_led:
            return

        (x, y, z, r, g, b) = (
            msg.point_stamped.point.x,
            msg.point_stamped.point.y,
            msg.point_stamped.point.z,
            msg.color_rgba.r,
            msg.color_rgba.g,
            msg.color_rgba.b)

        if r < 255 and g < 255 and b < 255:
            rospy.logwarn('No color %d %d %d', r, g, b)
            return

        (r, g, b) = (r / 255.0, g / 255.0, b / 255.0)

        self.publish_console_msg(x, y, z, r, g, b)

    def publish_fallback(self):
        rospy.loginfo('Publish fallback LED location')
        self.publish_console_msg(2.6, 0.0, 0.0, 1.0, 1.0, 1.0)
        
    def publish_console_msg(self, x, y, z, r, g, b):
        rospy.loginfo('Publish LED (%f, %f, %f, %d %d %d)', x, y, z, r, g, b)
        console_msg = Console(x=x, y=y, z=z, r=r, g=g, b=b)
        self.console_publisher.publish(console_msg)
        self.published_led = True

    def log_state(self):
        rospy.loginfo('State is %s', self.state)
        
if __name__ == '__main__':
    rospy.init_node('qual1_state')
    state = State()
    rospy.loginfo('Ready')
    state.log_state()
    rospy.spin()

