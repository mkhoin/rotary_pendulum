from environment import Env
import pylab
import numpy as np
from keras.layers import Dense
from keras.models import Sequential
from keras.optimizers import Adam
from keras import backend as K
from datetime import datetime
import sys
import random
import time

EPISODES = 2000

class A2CAgent:
    def __init__(self, state_size, action_size):
        self.load_model = False

        # 상태와 행동의 크기 정의
        self.state_size = state_size
        self.action_size = action_size
        self.value_size = 1

        # 액터-크리틱 하이퍼파라미터
        self.discount_factor = 0.99
        self.actor_lr = 0.001
        self.critic_lr = 0.005

        # 정책신경망과 가치신경망 생성
        self.actor = self.build_actor()
        self.critic = self.build_critic()
        self.actor_updater = self.actor_optimizer()
        self.critic_updater = self.critic_optimizer()

        if self.load_model:
            self.actor.load_weights("./save/pendulum_actor_trained.h5")
            self.critic.load_weights("./save/pendulum_critic_trained.h5")

    # actor: 상태를 받아 각 행동의 확률을 계산
    def build_actor(self):
        actor = Sequential()
        actor.add(Dense(24, input_dim=self.state_size, activation='relu', kernel_initializer='he_uniform'))
        actor.add(Dense(self.action_size, activation='softmax', kernel_initializer='he_uniform'))
        actor.summary()
        return actor

    # critic: 상태를 받아서 상태의 가치를 계산
    def build_critic(self):
        critic = Sequential()
        critic.add(Dense(24, input_dim=self.state_size, activation='relu', kernel_initializer='he_uniform'))
        critic.add(Dense(24, input_dim=self.state_size, activation='relu', kernel_initializer='he_uniform'))
        critic.add(Dense(self.value_size, activation='linear', kernel_initializer='he_uniform'))
        critic.summary()
        return critic

    # 정책신경망의 출력을 받아 확률적으로 행동을 선택
    def get_action(self, state):
        policy = self.actor.predict(state, batch_size=1).flatten()
        action = np.random.choice(self.action_size, 1, p=policy)[0]
        return action

    # 정책신경망을 업데이트하는 함수
    def actor_optimizer(self):
        action = K.placeholder(shape=[None, self.action_size])
        advantage = K.placeholder(shape=[None, ])

        action_prob = K.sum(action * self.actor.output, axis=1)
        cross_entropy = K.log(action_prob) * advantage
        loss = -K.sum(cross_entropy)

        optimizer = Adam(lr=self.actor_lr)
        updates = optimizer.get_updates(self.actor.trainable_weights, [], loss)
        train = K.function([self.actor.input, action, advantage], [], updates=updates)
        return train

    # 가치신경망을 업데이트하는 함수
    def critic_optimizer(self):
        target = K.placeholder(shape=[None, ])

        loss = K.mean(K.square(target - self.critic.output))

        optimizer = Adam(lr=self.critic_lr)
        updates = optimizer.get_updates(self.critic.trainable_weights, [], loss)
        train = K.function([self.critic.input, target], [], updates=updates)

        return train

    # 각 타임스텝마다 정책신경망과 가치신경망을 업데이트
    def train_model(self, state, action, reward, next_state, done):
        value = self.critic.predict(state)[0]
        next_value = self.critic.predict(next_state)[0]

        act = np.zeros([1, self.action_size])
        act[0][action] = 1

        # 벨만 기대 방정식를 이용한 어드벤티지와 업데이트 타깃
        if done:
            advantage = reward - value
            target = [reward]
        else:
            advantage = (reward + self.discount_factor * next_value) - value
            target = reward + self.discount_factor * next_value
        self.actor_updater([state, act, advantage])
        self.critic_updater([state, target])


if __name__ == "__main__":
    print(str(datetime.now()) + ' started')
    env = Env()

    # print(env.observation_space_shape)
    # print(env.action_space_shape)

    state_size = env.observation_space_shape[0]
    action_size = env.action_space_shape[0]

    # 액터-크리틱(A2C) 에이전트 생성
    agent = A2CAgent(state_size, action_size)

    scores, episodes = [], []

    try:
        for e in range(EPISODES):
            done = False
            score = 0
            state = env.reset()
            print("state : ", state)
            state = np.reshape(state, [1, state_size])

            while not done:
                action = agent.get_action(state)
                #real_action = int(np.array([(action - (action_size - 1) / 2) / ((action_size - 1) / 4)]))
                real_action = (action - 5) * 10
                next_state, reward, done = env.step(real_action)
                next_state = np.reshape(next_state, [1, state_size])

                now = datetime.now()
                print(now)

                print("action: {0} -->\nnext State: {1}\nreward: {2}\ndone: {3}\n".format(
                    action,
                    next_state,
                    reward,
                    done
                ))

                agent.train_model(state, action, reward, next_state, done)

                score += reward
                state = next_state

                if done:
                    scores.append(score)
                    episodes.append(e)
                    pylab.plot(episodes, scores, 'b')
                    pylab.savefig("./save/pendulum_a2c.png")
                    print("episode:", e, "  score:", score)

                    if 0 <= score:
                        agent.actor.save_weights("./save/pendulum_actor.h5")
                        agent.critic.save_weights("./save/pendulum_critic.h5")
                        max_score = score
                    # 이전 10개 에피소드의 점수 평균이 490보다 크면 학습 중단
                    if np.mean(scores[-min(10, len(scores)):]) > 190:
                        agent.actor.save_weights("./save/pendulum_actor.h5")
                        agent.critic.save_weights("./save/pendulum_critic.h5")
                        env.close()
                        sys.exit()
        env.close()

    except KeyboardInterrupt as e:
        env.close()

#     MAX_NUM_STEP = 1000

#     try:
#         for step in range(MAX_NUM_STEP):
#             if step % 2 == 0:
#                 action = random.randrange(0, 100)
#             else:
#                 action = random.randrange(-100, 0)
#
#             next_state, reward, done = env.step(action)
#
#             now = datetime.now()
#             print(now)
#
#             print("action: {0} -->\nnext State: {1}\nreward: {2}\ndone: {3}\n".format(
#                 action,
#                 next_state,
#                 reward,
#                 done
#             ))
#
#             time.sleep(0.1)
#
#         env.close()
#     except KeyboardInterrupt as e:
#         env.close()