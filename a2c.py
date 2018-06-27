import gym
import environment as Env
import pylab
import numpy as np
from keras.layers import Dense
from keras.models import Sequential
from keras.optimizers import Adam
from keras import backend as K
from datetime import datetime

if __name__ == "__main__":
    print(str(datetime.now()) + ' started')
    env = Env()
    env.reset()
    print(env.observation_space_shape)
    print(env.action_space_shape)
    # print(env.env.action_space.low)
    # print(env.env.action_space.high)