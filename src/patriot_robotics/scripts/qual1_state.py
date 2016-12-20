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
        self.published_led = False
        rospy.Subscriber('acquired', Empty, self.acquired)
        rospy.Subscriber('lost', Empty, self.lost)
        rospy.Subscriber('led_3D_location', PointStampedColorRGBA, 
                                               self.led_3D_location)
        
        self.console_publisher = rospy.Publisher("/srcsim/qual1/light", 
                                            Console, queue_size = 20)
        self.seeking = 'seeking'
        self.tracking = 'tracking'
        self.state = self.seeking

    def acquired(self, msg):
        if self.state == self.seeking:
            self.state= self.tracking
            self.log_state()

    def lost(self, msg):
        if self.state == self.tracking:
            self.state= self.seeking
            self.published_led = False
            self.log_state()

    def led_3D_location(self, msg):
        (x, y, z) = (
            msg.point_stamped.point.x,
            msg.point_stamped.point.y,
            msg.point_stamped.point.z)
        # RGB values in range [0-1]
        (r, g, b) = (
            msg.color_rgba.r,
            msg.color_rgba.g,
            msg.color_rgba.b)

        # TODO missing [0-255] to [0-1] range conversion
        rospy.loginfo('Publish LED (%f, %f, %f, %d %d %d)', x, y, z, r, g, b)
        

        console_msg = Console(x=x, y=y, z=z, r=r, g=g, b=b)
        self.console_publisher.publish(console_msg)
        self.published_led = True

    def log_state(self):
        rospy.loginfo('State is %s' % self.state)
        
if __name__ == '__main__':
    rospy.init_node('qual1_state')
    state = State()
    rospy.loginfo('Ready')
    state.log_state()
    rospy.spin()

