import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.optimizers import Adam


class Critic:
    def __init__(self, state_size, action_size, learning_rate=0.0001, filename='./critic_model.h5'):
        self.filename = filename
        self.state_size = state_size
        self.action_size = action_size
        self.learning_rate = learning_rate
        self.model = self._build_model()

    def _build_model(self):
        # State as input
        state_input = tf.keras.layers.Input(shape=(self.state_size,))
        state_h1 = Dense(400, activation='relu')(state_input)
        state_h2 = Dense(300, activation='relu')(state_h1)

        # Action as input
        action_input = tf.keras.layers.Input(shape=(self.action_size,))
        action_h1 = Dense(300, activation='relu')(action_input)

        # Combine state and action
        merged = tf.keras.layers.Concatenate()([state_h2, action_h1])

        # Final output layer
        output = Dense(1, activation='linear')(merged)

        model = tf.keras.models.Model(inputs=[state_input, action_input], outputs=output)
        model.compile(loss='mse', optimizer=Adam(lr=self.learning_rate))

        return model

    def train(self, states, actions, target_q_values):
        with tf.GradientTape() as tape:
            q_values = self.model([states, actions], training=True)
            loss = tf.keras.losses.MSE(target_q_values, q_values)
        grads = tape.gradient(loss, self.model.trainable_variables)
        self.model.optimizer.apply_gradients(zip(grads, self.model.trainable_variables))

    def save(self):
        self.model.save_weights(self.filename)

    def load(self):
        self.model.load_weights(self.filename)