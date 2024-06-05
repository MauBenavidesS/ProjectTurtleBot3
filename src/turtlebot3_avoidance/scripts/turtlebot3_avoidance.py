#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callbackLaserScan(data):
    regions = {
        'left':   data.ranges[330],
        'front-left':   data.ranges[350],
        'front':  data.ranges[0],
        'front-right':  data.ranges[10],
        'right':  data.ranges[30],
    }

    take_action(regions)

def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    print("front: {0:.2f}, left: {1:.2f}, right: {2:.2f}".format(regions['front'], regions['left'], regions['right']))

    # Adjusting thresholds for more responsive obstacle avoidance
    side_threshold = 0.6

    if regions['left'] < side_threshold:
        state_description = 'case 4 - Turn Left'
        linear_x = 0
        angular_z = 0.75
    elif regions['right'] < side_threshold:
        state_description = 'case 5 - Turn Right'
        linear_x = 0
        angular_z = -0.75
    elif regions['front-left'] < side_threshold:
        state_description = 'case 2 - Sharp Left'
        linear_x = 0.2
        angular_z = 0.75
    elif regions['front-right'] < side_threshold:
        state_description = 'case 3 - Sharp Right'
        linear_x = 0.2
        angular_z = -0.75
    else:
        state_description = 'case 0 - advance'
        linear_x = 0.5
        angular_z = 0

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('reading_laser')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/scan', LaserScan, callbackLaserScan)
    rospy.spin()
