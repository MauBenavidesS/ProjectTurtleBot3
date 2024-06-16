import tensorflow as tf
from tensorflow.keras import layers
import numpy as np
from turtlebot_env import TurtleBotEnv

# Define the Q-network
def build_model(state_shape, action_shape):
    model = tf.keras.Sequential()
    model.add(layers.InputLayer(state_shape))
    model.add(layers.Dense(24, activation='relu'))
    model.add(layers.Dense(24, activation='relu'))
    model.add(layers.Dense(action_shape, activation='linear'))
    return model

# Define the DQN agent
class DQNAgent:
    def __init__(self, state_shape, action_shape):
        self.model = build_model(state_shape, action_shape)
        self.target_model = build_model(state_shape, action_shape)
        self.update_target_network()
        
        self.optimizer = tf.keras.optimizers.Adam(learning_rate=0.001)
        self.loss_function = tf.keras.losses.MeanSquaredError()
        
        self.replay_buffer = []
        self.buffer_size = 10000
        self.batch_size = 64
        self.gamma = 0.99
        self.epsilon = 1.0
        self.epsilon_min = 0.1
        self.epsilon_decay = 0.995

        # Compile the model immediately after building
        self.compile_model()

    def update_target_network(self):
        self.target_model.set_weights(self.model.get_weights())
    
    def get_action(self, state):
        if np.random.rand() < self.epsilon:
            return np.random.randint(5)
        q_values = self.model.predict(state[np.newaxis])
        return np.argmax(q_values[0])
    
    def train(self, state, action, reward, next_state, done):
        self.replay_buffer.append((state, action, reward, next_state, done))
        if len(self.replay_buffer) > self.buffer_size:
            self.replay_buffer.pop(0)
        
        if len(self.replay_buffer) < self.batch_size:
            return
        
        minibatch = np.random.choice(len(self.replay_buffer), self.batch_size, replace=False)
        for i in minibatch:
            state, action, reward, next_state, done = self.replay_buffer[i]
            target = reward
            if not done:
                target += self.gamma * np.amax(self.target_model.predict(next_state[np.newaxis])[0])
            target_f = self.model.predict(state[np.newaxis])
            target_f[0][action] = target
            self.model.fit(state[np.newaxis], target_f, epochs=1, verbose=0)
        
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
    
    def compile_model(self):
        self.model.compile(optimizer='adam', loss='mse')  # Example optimizer and loss

    def save_model(self, filepath):
        # Save model to disk
        self.model.save(filepath)

    def load_model(self, filepath):
        # Load model from disk
        self.model = load_model(filepath)

# Train the agent
agent = DQNAgent((5,), 5)
env = TurtleBotEnv()

episodes = 100
for e in range(episodes):
    print("Episode:", e)
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

# Save the model after training
agent.save_model('trained_model_10_ep')  # Example filename without extension (.h5 or SavedModel format)
