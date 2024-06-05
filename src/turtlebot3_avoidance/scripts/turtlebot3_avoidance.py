#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callbackLaserScan(data):
    # regions = {
    #     'right':  min(min(data.ranges[0:143]), 10),
    #     'front':  min(min(data.ranges[144:287]), 10),
    #     'left':   min(min(data.ranges[288:431]), 10),
    # }
    regions = {
        'left':   data.ranges[320],
        'front-left':   data.ranges[340],
        'front':  data.ranges[0],
        'front-right':  data.ranges[20],
        'right':  data.ranges[40],
    }

    take_action(regions)

def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    print("front: {0:.2f}, left: {1:.2f}, right: {2:.2f}".format(regions['front'], regions['left'], regions['right']))

    # Adjusting thresholds for more responsive obstacle avoidance
    front_threshold = 0.7
    side_threshold = 0.4

    if regions['front'] < front_threshold:
        state_description = 'case 1 - turn'
        linear_x = 0
        angular_z = 1
    elif regions['front-left'] < side_threshold:
        state_description = 'case 2 - right'
        linear_x = 0
        angular_z = 0.75
    elif regions['front-right'] < side_threshold:
        state_description = 'case 3 - right'
        linear_x = 0
        angular_z = -0.75
    elif regions['left'] < side_threshold:
        state_description = 'case 4 - right'
        linear_x = 0
        angular_z = 0.5
    elif regions['right'] < side_threshold:
        state_description = 'case 5 - right'
        linear_x = 0
        angular_z = -0.5
    else:
        state_description = 'case 0 - advance'
        linear_x = 0.5
        angular_z = 0
        # rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('reading_laser')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/scan', LaserScan, callbackLaserScan)
    rospy.spin()
