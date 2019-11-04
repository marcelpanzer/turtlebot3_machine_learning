import random
import numpy as np
import time

from collections import deque
from sumtree import SumTree

class MemoryBuffer(object):
    """ Memory Buffer Helper class for Experience Replay
    using a double-ended queue or a Sum Tree (for PER)
    """
    def __init__(self, buffer_size, with_per = False):
        """ Initialization
        """
            # Prioritized Experience Replay
        self.alpha = 0.5
        self.epsilon = 0.01
        self.buffer = SumTree(buffer_size)

        self.count = 0
        self.with_per = with_per
        self.buffer_size = buffer_size

    def memorize(self, state, action, reward, done, new_state, error=None):
        """ Save an experience to memory, optionally with its TD-Error
        """

        experience = (state, action, reward, done, new_state)
        if(self.with_per):
            priority = self.priority(error[0])
            self.buffer.add(priority, experience)
            self.count += 1
        else:
            # Check if buffer is already full
            if self.count < self.buffer_size:
                self.buffer.append(experience)
                self.count += 1
            else:
                self.buffer.popleft()
                self.buffer.append(experience)

    def priority(self, error):
        """ Compute an experience priority, as per Schaul et al.
        """
        return (error + self.epsilon) ** self.alpha

    def size(self):
        """ Current Buffer Occupation
        """
        return self.count

    def sample_batch(self, batch_size):
        """ Sample a batch, optionally with (PER)
        """
        batch = []
        s_batch = []
        a_batch = []
        r_batch = []
        d_batch = []
        new_s_batch = []
        idx_ = []
        # Sample using prorities
        if(self.with_per):
            T = self.buffer.total() // batch_size
            for i in range(batch_size):
                sum = []
                a, b = T * i, T * (i + 1)
                s = random.uniform(a, b)
                idx, error, data = self.buffer.get(s)
                # batch.append((*data, idx))
                s_batch.append(data[0])
                a_batch.append(data[1])
                r_batch.append(data[2])
                d_batch.append(data[3])
                new_s_batch.append(data[4])
                idx_.append(idx)

        return s_batch, a_batch, r_batch, d_batch, new_s_batch, idx_

    def update(self, idx, new_error):
        """ Update priority for idx (PER)
        """
        self.buffer.update(idx, self.priority(new_error))

    def clear(self):
        """ Clear buffer / Sum Tree
        """
        if(self.with_per): self.buffer = SumTree(buffer_size)
        else: self.buffer = deque()
        self.count = 0
