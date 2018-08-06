from environment import Env
# import pylab
# import numpy as np
# from keras.layers import Dense
# from keras.models import Sequential
# from keras.optimizers import Adam
# from keras import backend as K
from datetime import datetime
import random
import time

if __name__ == "__main__":
    print(str(datetime.now()) + ' started')
    env = Env()
    env.reset()
    print(env.observation_space_shape)
    print(env.action_space_shape)

    MAX_NUM_STEP = 1000

    try:
        for step in range(MAX_NUM_STEP):
            if step % 2 == 0:
                action = random.randrange(0, 100)
            else:
                action = random.randrange(-100, 0)

            next_state, reward, done = env.step(action)

            #now = datetime.now()
            #print(now)

            print("action: {0}, next State: {1}, reward: {2}, done: {3}".format(
                action,
                next_state,
                reward,
                done
            ))

            #time.sleep(1)

        env.close()
    except KeyboardInterrupt as e:
        env.close()