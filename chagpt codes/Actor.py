
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.optimizers import Adam


class Actor:
    def __init__(self, state_size, action_size, learning_rate=0.005, filename='./actor_model.h5'):
        self.filename = filename
        self.state_size = state_size
        self.action_size = action_size
        self.learning_rate = learning_rate
        self.model = self._build_model()

    def _build_model(self):
        model = tf.keras.models.Sequential()
        model.add(tf.keras.layers.Dense(400, input_dim=self.state_size, activation='relu'))
        model.add(tf.keras.layers.Dense(300, activation='relu'))
        model.add(tf.keras.layers.Dense(self.action_size, activation='tanh'))
        model.compile(loss='mse', optimizer=tf.keras.optimizers.Adam(lr=self.learning_rate))
        return model

    def train(self, states, actions_grads):
        with tf.GradientTape() as tape:
            actions = self.model(states)
            loss = tf.reduce_mean(tf.multiply(actions, actions_grads))
        grads = tape.gradient(loss, self.model.trainable_variables)
        self.model.optimizer.apply_gradients(zip(grads, self.model.trainable_variables))

    def save(self):
        self.model.save_weights(self.filename)

    def load(self):
        self.model.load_weights(self.filename)