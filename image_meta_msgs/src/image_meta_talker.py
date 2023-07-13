#!/usr/bin/env python

## Simple talker demo that publishes image_meta_msgs/ImageMeta messages
## to the 'chatter' topic

import rospy
from image_meta_msgs.msg import *
from std_msgs.msg import *

def talker():
    pub = rospy.Publisher('chatter', ImageMeta, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    
    rate = rospy.Rate(10) # 10hz

    msg_seq = 0
    hw_drv_delta = rospy.Duration.from_sec(0.001)

    while not rospy.is_shutdown():
        msg = ImageMeta()

        now = rospy.Time.now()

        msg.hardware_header = std_msgs.msg.Header(msg_seq, now - hw_drv_delta, 'hardware')
        msg.driver_header = std_msgs.msg.Header(msg_seq, now, 'driver')

        msg_seq = msg_seq + 1

        msg.name_value_floats = [NameValueFloat('float0', 1.234)]
        msg.name_value_ints = [NameValueInt('int0', 1234567890)]
        msg.name_value_strings = [NameValueString('string0', 'Hello, World!')]
        
        pub.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
