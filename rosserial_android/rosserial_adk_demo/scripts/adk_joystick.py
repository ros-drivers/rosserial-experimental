#! /usr/bin/env python

import roslib; roslib.load_manifest('rosserial_adk_demo')
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def deadband(val, dead_band):
    if ( val < dead_band) and (val > -dead_band):
        val = 0
    if (val > 1):
        val = 1
    if (val < -1):
        val = -1
    return val

if __name__ ==  '__main__':
    rospy.init_node('adk_joystick')
    
    pub = rospy.Publisher("cmd_vel", Twist)
    x, t = 0, 0
    def joy_cb(msg):
        x += deadband(msg.axes[0], 0.25)
        t += deadband(msg.axes[1], 0.25)*0.5;
        x /=2
        t /=2
        cmd.linear.x = x
        cmd.angular.z = t;
        pub.publish(cmd)
    
    sub = rospy.Subscriber("joy", Joy, joy_cb);
    
    rospy.spin()
        
    
