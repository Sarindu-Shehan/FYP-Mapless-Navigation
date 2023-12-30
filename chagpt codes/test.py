import numpy as np
from tensorflow.keras.models import load_model
import myenvironment  # Replace with your actual environment module

def test_agent(env, actor, num_episodes=10):
    total_reward = 0.0
    for _ in range(num_episodes):
        state = env.reset()
        episode_reward = 0.0
        done = False

        while not done:
            # Assuming state is a tuple and the relevant data is in the third element
            formatted_state = np.array([state[2]]) if isinstance(state, tuple) else np.array([state])
            
            action = actor.predict(formatted_state)[0]
            next_state, reward, done, _ = env.step(action)
            episode_reward += reward
            state = next_state

        total_reward += episode_reward
        print(f"Episode Reward: {episode_reward}")

    avg_reward = total_reward / num_episodes
    print(f"Average Reward over {num_episodes} episodes: {avg_reward}")
    return avg_reward


def main():
    # Load the environment
    env = myenvironment.Environment()  # Make sure this matches your environment's initialization

    # Load the trained actor model
    actor_model_path = './actor_model.h5'
    actor = load_model(actor_model_path)

    # Test the agent
    test_agent(env, actor, num_episodes=10)

if __name__ == "__main__":
    main()
