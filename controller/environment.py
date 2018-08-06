import threading
import math
from enum import Enum
from collections import deque
import time
import random
from datetime import datetime
import paho.mqtt.client as mqtt
import numpy as np

MQTT_SERVER = 'localhost'
MQTT_PENDULUM_ANGLE_TOPIC = 'Pendulum/angle'
MQTT_MOTOR_ANGLE_TOPIC = 'Motor/angle'
MQTT_MOTOR_POWER_TOPIC = 'Motor/power'

class Device(Enum):
    pendulum = 1
    motor = 2

self_env = None

class Env:
    def __init__(self):
        global self_env
        self.pendulum_info_buffer = deque()
        self.motor_info_buffer = deque()
        self.state_space_shape = (5,)
        self.action_space_shape = (11,)

        self.sub = mqtt.Client(client_id="angle_sub", transport="UDP")
        self.sub.on_connect = self.__on_connect
        self.sub.on_message = self.__on_message
        self.sub.connect(MQTT_SERVER, 1883, 60)

        s = threading.Thread(target=self.__sub, args=(self.sub,))
        s.daemon = True
        s.start()

        self.pub = mqtt.Client(client_id="motor_power_pub", transport="UDP")
        self.pub.connect(MQTT_SERVER, 1883, 60)

        self.steps = 0
        self.done = False
        self.rewards = []
        self.info = None
        self_env = self

    @staticmethod
    def __on_connect(client, userdata, flags, rc):
        print("mqtt broker connected with result code " + str(rc))
        client.subscribe(topic=MQTT_PENDULUM_ANGLE_TOPIC)
        client.subscribe(topic=MQTT_MOTOR_ANGLE_TOPIC)

    @staticmethod
    def __on_message(client, userdata, msg):
        if msg.topic == MQTT_PENDULUM_ANGLE_TOPIC:
            pendulum_info = str(msg.payload)
            msg = pendulum_info.split(":")
            theta = int(msg[1].split(',')[0])
            theta_dot = int(msg[2].split("'")[0])
            self_env.__insert_angle_info_to_buffer(device=Device.pendulum, theta=theta, theta_dot=theta_dot)
            #print("Pendulum Angle value: {0}, {1}".format(angle, speed))
        elif msg.topic == MQTT_MOTOR_ANGLE_TOPIC:
            motor_info = str(msg.payload)
            msg = motor_info.split(":")
            theta = int(msg[1].split("'")[0])
            self_env.__insert_angle_info_to_buffer(device=Device.motor, theta=theta, theta_dot=0)
            #print("Motor Angle value: {0}, {1}".format(angle, speed))

    def __insert_angle_info_to_buffer(self, device, theta, theta_dot):
        cosine_theta = math.cos(float(theta))
        sine_theta = math.sin(float(theta))
        rad = math.radians(theta)
        time_epoch = datetime.now().timestamp()
        if device == Device.pendulum:
            self.pendulum_info_buffer.append((time_epoch, rad, [cosine_theta, sine_theta, theta_dot / 20]))
        elif device == Device.motor:
            self.motor_info_buffer.append((time_epoch, theta, [cosine_theta, sine_theta]))
        else:
            raise AttributeError("Unrecognized Device")

    @staticmethod
    def __sub(sub):
        try:
            print("Sub thread started!")
            sub.loop_forever()
        except KeyboardInterrupt:
            print("Sub Interrupted!")
            sub.unsubscribe([MQTT_PENDULUM_ANGLE_TOPIC, MQTT_MOTOR_ANGLE_TOPIC])


    ################################################
    ###  Environment Methods: reset, step, close ###
    ################################################
    def reset(self):
        self.steps = 0
        self.done = False
        del self.rewards[:]

        # reset position
        self.pub.publish(topic=MQTT_MOTOR_POWER_TOPIC, payload="speed:" + str(999999))

        # wait while buffers are not empty
        while True:
            if len(self.pendulum_info_buffer) != 0 and len(self.motor_info_buffer) != 0:
                break
            time.sleep(0.01)

        # pendulum_info = [time_epoch, radian, [cos(theta), sin(theta), theta_dot]]
        pendulum_info = self.pendulum_info_buffer.pop()
        # motor_info = [time_epoch, theta, [cos(theta), sin(theta)]]
        motor_info = self.motor_info_buffer.pop()

        # return [cos(p_theta), sin(p_theta), p_theta_dot, cos(m_theta), sin(m_theta)]
        return pendulum_info[2] + motor_info[2]

    def step(self, action):
        self.steps += 1

        # action
        self.pub.publish(topic=MQTT_MOTOR_POWER_TOPIC, payload="speed:" + str(action))

        # wait while buffers are not empty
        while True:
            if len(self.pendulum_info_buffer) != 0 and len(self.motor_info_buffer) != 0:
                break
            time.sleep(0.01)
            # print("step:  {0}  {1}".format(len(self.pendulum_angle_buffer), len(self.motor_angle_buffer)))

        # pendulum_info = [time_epoch, radian, [cos(theta), sin(theta), theta_dot]]
        pendulum_info = self.pendulum_info_buffer.pop()
        # motor_info = [time_epoch, theta, [cos(theta), sin(theta)]]
        motor_info = self.motor_info_buffer.pop()

        # action normalization (-2 ~ 2)
        n_action = action / 50.0
        # reward = -(pendulum_radian^2 + 0.1 * pendulum_theta_dot^2 + 0.01 * action^2)
        reward = -((pendulum_info[1] ** 2) + (0.1 * (pendulum_info[2][2] ** 2)) + (0.01 * (n_action ** 2)))

        self.rewards.append(reward)

        self.isDone(motor_info[1])

        # pendulum_angle_time_epoch = datetime.fromtimestamp(
        #     pendulum_angle[0]
        # ).strftime('%Y-%m-%d %H:%M:%S')

        # motor_angle_time_epoch = datetime.fromtimestamp(
        #     motor_angle[0]
        # ).strftime('%Y-%m-%d %H:%M:%S')

        # print("Inside step: pendulum_angle_time_epoch - {0}, motor_angle_time_epoch - {1}". format(
        #     pendulum_angle_time_epoch,
        #     motor_angle_time_epoch
        # ))

        # return state, reward, done, info
        return pendulum_info[2] + motor_info[2], reward, self.done, self.info

    def close(self):
        self.pub.publish(topic=MQTT_MOTOR_POWER_TOPIC, payload="speed:0")   # set motor speed 0

    def isDone(self, motor_pos):
        if self.steps > 300:                        # maximum step
            self.info = "max step"
            self.done = True
        elif np.mean(self.rewards[-30:]) > -0.1:    # success
            self.info = "success"
            self.done = True
        elif motor_pos > 360 or motor_pos < -360:   # limit motor position
            self.info = "limit position"
            self.done = True
        else:
            self.done = False