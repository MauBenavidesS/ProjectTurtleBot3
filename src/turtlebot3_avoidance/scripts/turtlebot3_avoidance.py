#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('obstacle_avoidance')

        # Create a publisher for the /cmd_vel topic to send velocity commands
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Subscribe to the /scan topic to receive laser scan data
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callbackLaserScan)

        # Keep the program alive and listening for data
        rospy.spin()

    def callbackLaserScan(self, data):
        """
        This function processes the incoming laser scan data. It categorizes the distance data into different 
        regions (left, front-left, front, front-right, right) and calls the moveRobot function to decide the 
        robot's movements based on this data.
        """
        # Determine the minimum distance to an obstacle in each region for obstacle avoidance
        regions = {
            'left':   min(min(data.ranges[330:335]), 10),
            'front-left':   min(min(data.ranges[350:355]), 10),
            'front':  min(min(data.ranges[0:5] + data.ranges[355:360]), 10),
            'front-right':  min(min(data.ranges[5:10]), 10),
            'right':  min(min(data.ranges[25:30]), 10),
        }

        # Call the moveRobot function passing to it the Lidar scan obstacle data
        self.moveRobot(regions)

    def moveRobot(self, regions):
        """
        This function determines the robot's movements based on the distance data from different regions. It sets 
        linear and angular velocities to avoid obstacles and publishes the movement commands.
        """
        # Initialize Twist message for velocity commands
        msg = Twist()
        linear_x = 0
        angular_z = 0
        state_description = ''

        # Log the lidar scan data
        rospy.loginfo("--Lidar Scan Data-- Front: {0:.2f}, Left: {1:.2f}, Right: {2:.2f}".format(regions['front'], regions['left'], regions['right']))

        # Thresholds adjustment for more responsive obstacle avoidance
        side_threshold = 0.6

        # Decide movement based on obstacle distances in each region
        if regions['left'] < side_threshold:
            state_description = 'Case 1 - Turn Left'
            linear_x = 0
            angular_z = 0.75
        elif regions['right'] < side_threshold:
            state_description = 'Case 2 - Turn Right'
            linear_x = 0
            angular_z = -0.75
        elif regions['front-left'] < side_threshold:
            state_description = 'Case 3 - Sharp Left'
            linear_x = 0.2
            angular_z = 0.75
        elif regions['front-right'] < side_threshold:
            state_description = 'Case 4 - Sharp Right'
            linear_x = 0.2
            angular_z = -0.75
        else:
            state_description = 'Case 0 - Advance'
            linear_x = 0.5
            angular_z = 0

        # Log the current state for debugging
        rospy.loginfo(state_description)

        # Set the velocity commands in the Twist message
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        # Publish the velocity commands to the /cmd_vel topic
        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        ObstacleAvoidance()
    except rospy.ROSInterruptException:
        pass