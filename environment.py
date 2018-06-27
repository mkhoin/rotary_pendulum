import threading
import math
from enum import Enum
from collections import deque
import time
import random
import datetime


class Device(Enum):
    pendulum = 1
    motor = 2


class Env:
    def __init__(self):
        self.pendulum_angle_buffer = deque()
        self.motor_angle_buffer = deque()
        self.action_id = 0
        self.observation_space_shape = (6,)
        self.action_space_shape = (20,)

    def __insert_angle_info_to_buffer(self, device, angle, theta):
        cosine_angle = math.cos(angle)
        sine_angle = math.sin(angle)
        time_epoch = datetime.datetime.now().timestamp()
        if device == Device.pendulum:
            self.pendulum_angle_buffer.append([time_epoch, cosine_angle, sine_angle, theta])
        elif device == Device.motor:
            self.motor_angle_buffer.append([time_epoch, cosine_angle, sine_angle, theta])
        else:
            raise AttributeError("Unrecognized Device")

    @staticmethod
    def __simulated_angle_generation(env):
        device_list = [Device.pendulum, Device.motor]
        while True:
            device = random.choice(device_list)
            angle = random.randrange(-100, 100)
            theta = random.randrange(-10, 10)
            env.__insert_angle_info_to_buffer(device=device, angle=angle, theta=theta)
            sleep_time = random.randrange(0, 10)
            time.sleep(sleep_time / 100.0)
            print("insert - {0}, {1}, {2}".format(device, angle, theta))

    def reset(self):
        self.action_id = 0
        t = threading.Thread(target=self.__simulated_angle_generation, args=(self,))
        t.daemon = True
        t.start()

    def step(self, action):
        pendulum_angle = None
        motor_angle = None

        while True:
            if len(self.pendulum_angle_buffer) != 0 and len(self.motor_angle_buffer) != 0:
                break

        if len(self.pendulum_angle_buffer) != 0 and len(self.motor_angle_buffer) != 0:
            pendulum_angle = self.pendulum_angle_buffer.popleft()
            motor_angle = self.motor_angle_buffer.popleft()
        return pendulum_angle + motor_angle, None, None

if __name__ == "__main__":

    env = Env()

    env.reset()
    print("Main Thread")

    for _ in range(100):
        action = random.randrange(-10, 10)
        next_state, reward, done = env.step(action)

        next_state[0] = datetime.datetime.fromtimestamp(
            next_state[0]
        ).strftime('%Y-%m-%d %H:%M:%S')

        next_state[4] = datetime.datetime.fromtimestamp(
            next_state[4]
        ).strftime('%Y-%m-%d %H:%M:%S')

        print("\naction: {0} -->\nnext State: {1}\nreward: {2}\ndone: {3}\n".format(action, next_state, reward, done))
        time.sleep(1)

