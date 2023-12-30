from collections import deque
import random
import numpy as np

class ReplayBuffer(object):

    def __init__(self, buffer_size, random_seed=123):
        """
        The right side of the deque contains the most recent experiences 
        """
        self.buffer_size = buffer_size
        self.count = 0
        self.buffer = deque()
        random.seed(random_seed)

    def add(self, s, a, r, t, s2):
        experience = (s, a, r, t, s2)
        

        if self.count < self.buffer_size: 
            self.buffer.append(experience)
            self.count += 1
        else:
            self.buffer.popleft()
            self.buffer.append(experience)

    def size(self):
        return self.count

    def sample_batch(self, batch_size):
        batch = random.sample(self.buffer, min(len(self.buffer), batch_size))

        # Process each experience to extract the relevant state information
        s_batch = np.array([experience[0][2] for experience in batch], dtype='float32')  # Extracting the list of sensor readings
        a_batch = np.array([experience[1] for experience in batch], dtype='float32')
        r_batch = np.array([experience[2] for experience in batch], dtype='float32')
        s2_batch = np.array([experience[3][2] for experience in batch], dtype='float32')  # Extracting the list of sensor readings for next_state
        t_batch = np.array([experience[4] for experience in batch], dtype='bool')

        return s_batch, a_batch, r_batch, s2_batch, t_batch

    def clear(self):
        self.buffer.clear()
        self.count = 0