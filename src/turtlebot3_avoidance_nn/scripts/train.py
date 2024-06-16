import rospy
import numpy as np
from turtlebot_env import TurtleBotEnv
from dqn_agent import DQNAgent

if __name__ == '__main__':
    env = TurtleBotEnv()
    agent = DQNAgent((5,), 5)

    episodes = 1000
    for e in range(episodes):
        state = env.reset()
        done = False
        total_reward = 0
        while not done:
            action = agent.get_action(state)
            next_state, reward, done, _ = env.step(action)
            agent.train(state, action, reward, next_state, done)
            state = next_state
            total_reward += reward
        
        agent.update_target_network()
        print(f"Episode {e+1}/{episodes}, Total Reward: {total_reward}")
