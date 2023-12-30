import numpy as np
import tensorflow as tf
#from ddpg import Actor, Critic, ReplayBuffer  # Adjust the import paths as needed
from Actor import Actor
from Critic import Critic
from ReplayBuffer import ReplayBuffer
from myenvironment import Environment
from tensorflow.keras.models import load_model

# Set random seeds for reproducibility
np.random.seed(123)
tf.random.set_seed(123)

# Environment and DDPG parameters
state_dim = 16
action_dim = 1
max_action = 1.0  # Adjust based on your environment
actor_lr = 0.005
critic_lr = 0.0001
batch_size = 100
replay_buffer_size = 1000000
num_episodes = 2000  # Adjust based on your training needs

# Initialize environment, actor, critic, and replay buffer
env = Environment()
actor = Actor(state_dim, action_dim, actor_lr)
critic = Critic(state_dim, action_dim, critic_lr)
replay_buffer = ReplayBuffer(replay_buffer_size)


try:
    critic.load()
    actor.load()
except Exception as e:
    print(e.__repr__)

# Training loop
for episode in range(num_episodes):
    state = env.reset()
    episode_reward = 0

    while True:
        # Select action using the actor model
        # print("State:", state)

        # Extract the sensor readings from the state tuple
        sensor_readings = state[2]  # This is the list of sensor readings

        # Convert the sensor readings to a NumPy array and reshape
        state_array = np.array(sensor_readings, dtype=np.float32).reshape(1, -1)

        # Make a prediction using the actor model
        action = actor.model.predict(state_array)[0]
        print("Action:", action)

        # Execute the action and get the next state, reward, and done flag
        next_state, reward, done, _ = env.step(action)

        # Store experience in replay buffer
        replay_buffer.add(state, action, reward, next_state, done)

        # Train the actor and critic networks
        if replay_buffer.size() >= batch_size:
            states, actions, rewards, next_states, dones = replay_buffer.sample_batch(batch_size)
            # Calculate target Q-values and train the critic
            # ... (Critic training logic goes here)

            # Update the actor policy using the sampled policy gradient
            # ... (Actor training logic goes here)

        state = next_state
        episode_reward += reward

        if done:
            break

        

    print(f"Episode: {episode + 1}, Reward: {episode_reward}")

# Save models
actor.model.save('actor_model.h5')
critic.model.save('critic_model.h5')
