import gym
from gym import spaces
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

myNum = 45

class TurtleBotEnv(gym.Env):
    def __init__(self):
        super(TurtleBotEnv, self).__init__()
        
        # Define action and observation space
        self.action_space = spaces.Discrete(5)  # 5 actions: advance, turn left, turn right, sharp left, sharp right
        self.observation_space = spaces.Box(low=0, high=10, shape=(5,), dtype=np.float32)  # Lidar readings
        
        # Initialize ROS node
        rospy.init_node('turtlebot_rl', anonymous=True)
        
        # Create a publisher for the /cmd_vel topic to send velocity commands
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Subscribe to the /scan topic to receive laser scan data
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callbackLaserScan)
        
        self.laser_data = None
        self.state = np.zeros(5)  # Initialize state as an array of zeros
        self.reward = 0
        self.done = False
        
    def callbackLaserScan(self, data):
        self.laser_data = data.ranges
        self.state = np.array([min(min(data.ranges[330:335]), 10),
                               min(min(data.ranges[350:355]), 10),
                               min(min(data.ranges[0:5] + data.ranges[355:360]), 10),
                               min(min(data.ranges[5:10]), 10),
                               min(min(data.ranges[25:30]), 10)])
        
    def step(self, action):
        # Execute action
        msg = Twist()
        if action == 0:  # advance
            msg.linear.x = 0.5
            msg.angular.z = 0
        elif action == 1:  # turn left
            msg.linear.x = 0
            msg.angular.z = 0.75
        elif action == 2:  # turn right
            msg.linear.x = 0
            msg.angular.z = -0.75
        elif action == 3:  # sharp left
            msg.linear.x = 0.2
            msg.angular.z = 0.75
        elif action == 4:  # sharp right
            msg.linear.x = 0.2
            msg.angular.z = -0.75
        
        self.pub.publish(msg)
        
        # Calculate reward
        self.reward = 0
        side_threshold = 0.6
        for region in self.state:
            if region < side_threshold:
                self.reward -= 1
            if region < side_threshold / 2:
                self.reward -= 1
        if action == 0:
            self.reward += 1
        
        # Check if episode is done
        if min(self.state) < 0.2:  # collision threshold
            self.done = True
        
        return self.state, self.reward, self.done, {}
    
    def reset(self):
        # Reset state
        self.state = np.zeros(5)  # Reset state to zeros or any initial state representation
        self.reward = 0
        self.done = False
        rospy.sleep(1)  # give time for the turtlebot to stabilize
        return self.state
    
    def render(self, mode='human'):
        pass

env = TurtleBotEnv()