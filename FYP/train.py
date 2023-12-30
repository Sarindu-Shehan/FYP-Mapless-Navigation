import time
import numpy as np
import tensorflow as tf
tf.compat.v1.enable_eager_execution()
import ddpg
import os
import matplotlib.pyplot as plt
from myenvironment import Environment

TRAIN_MODE = False
n=1000
episode_list = list(range(n))
reward_list = []

try:  
    os.makedirs('./saved')
except OSError:  
    print ("Creation of the directory failed")
else:  
    print ("Successfully created the directory")

print(tf.__version__)
env = Environment()

# env.start_simulation()
# time.sleep(0.5)

critic = ddpg.Critic()
actor = ddpg.Actor()
target_critic = ddpg.TargetCritic()
target_actor = ddpg.TargetActor()

try:
    critic.load()
    actor.load()
except Exception as e:
    print(e.__repr__)

target_actor.hard_copy(actor.model.trainable_variables)
target_critic.hard_copy(critic.model.trainable_variables)

ou = ddpg.OrnsteinUhlenbeckActionNoise(mu=np.zeros(1,))
buffer = ddpg.ReplayBuffer(100000)
global ep_ave_max_q_value
ep_ave_max_q_value = 0
global total_reward
total_reward = 0

def start_new_episode():
    env.terminate_simulation()
    time.sleep(0.5)
    env.start_simulation()
    time.sleep(0.5)
    env.initialize_program()
    time.sleep(0.5)

def create_tensorboard():
    global_step = tf.compat.v1.train.get_or_create_global_step()

    logdir = "./logs/"
    writer = tf.contrib.summary.create_file_writer(logdir)
    writer.set_as_default()
    return global_step, writer

global global_step
global_step, writer = create_tensorboard()

def train(action, reward, state, state2, done):
    global ep_ave_max_q_value
    
    # state2 = np.array(state2,dtype="float32")
    # state = np.array(state,dtype="float32")
    # print("a",action,"a",reward,"a",done)
    buffer.add(state, action, [reward], [done], state2)
    batch_size = 150

    if buffer.size() > batch_size:
        s_batch, a_batch, r_batch, t_batch, s2_batch = buffer.sample_batch(batch_size)
        # print(s2_batch)
        target_action2 = target_actor.model.predict(s2_batch)
        predicted_q_value = target_critic.model.predict([s2_batch, target_action2])

        yi = []
        for i in range(batch_size):
            if t_batch[i]:
                yi.append(r_batch[i])
            else:
                yi.append(r_batch[i] + 0.99 * predicted_q_value[i])

        predictions,loss = critic.train_step(s_batch, a_batch, yi)

        # print("loss-",loss)

        ep_ave_max_q_value += np.amax(predictions)

        grad = critic.actor_gradient(s_batch, actor)
        actor.train_step(s_batch, grad)

        target_actor.update(actor.model.trainable_variables)
        target_critic.update(critic.model.trainable_variables)

for episode in range(n):
    global_step.assign_add(1)
    env.set_motor_speeds(1,1)
    dis_min,_,_,obs = env.observe()
    # print(np.array(obs))
    done = False
    j = 0
    
    
    ep_ave_max_q_value = 0
    total_reward = 0
    while not done:
        
        noise = ou()
        
        # print(obs)
        action = actor.model.predict(np.array([obs]))[0]
        
        if TRAIN_MODE:
            # print("bef noice-",action)
            action = action + noise
        # print(action)
        p_dis_min = dis_min
        obs2, reward, done, dis_min = env.step(action,p_dis_min)
        total_reward += reward
        # print("obs2 - ",obs2,"total reward - ",total_reward,"reward - ",reward,"done - ",done)

        if TRAIN_MODE:
            train(action, reward, obs, obs2, done)
        obs = obs2
        j += 1
        if j == 150:
            done = True
    print(buffer.size())    

    with writer.as_default(), tf.contrib.summary.always_record_summaries():
        tf.contrib.summary.scalar("average_max_q", ep_ave_max_q_value / float(j))
        tf.contrib.summary.scalar("reward", total_reward)
    # print(TRAIN_MODE)
    if TRAIN_MODE:
        critic.save()
        actor.save()
        print('average_max_q: ', ep_ave_max_q_value / float(j), 'reward: ', total_reward, 'episode:', episode)
        reward_list.append(total_reward)
    start_new_episode()


env.terminate_simulation()

# print(episode_list)

# plt.plot(episode_list, reward_list)

# plt.title('Reward Plot')
# plt.xlabel('Episodes')
# plt.ylabel('Reward')
 
# #show plot to user
# plt.show()