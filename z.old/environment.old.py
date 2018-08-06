import threading
import math
from enum import Enum
from collections import deque
import time
import random
from datetime import datetime
import paho.mqtt.client as mqtt

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
        self.pendulum_angle_buffer = deque()
        self.motor_angle_buffer = deque()
        self.action_id = 0
        self.observation_space_shape = (6,)
        self.action_space_shape = (20,)

        self.sub = mqtt.Client(client_id="angle_sub", transport="UDP")
        self.sub.on_connect = self.__on_connect
        self.sub.on_message = self.__on_message
        self.sub.connect(MQTT_SERVER, 1883, 60)

        self.pub = mqtt.Client(client_id="motor_power_pub", transport="UDP")
        self.pub.connect(MQTT_SERVER, 1883, 60)

        self.pub_to_EV3 = mqtt.Client(client_id="motor_power_pub_to_EV3", transport="UDP")
        self.pub_to_EV3.connect(MQTT_SERVER, 1883, 60)

        self_env = self

    @staticmethod
    def __on_connect(client, userdata, flags, rc):
        print("mqtt broker connected with result code " + str(rc))
        client.subscribe(topic=MQTT_PENDULUM_ANGLE_TOPIC)
        client.subscribe(topic=MQTT_MOTOR_ANGLE_TOPIC)

    @staticmethod
    def __on_message(client, userdata, msg):
        if msg.topic == MQTT_PENDULUM_ANGLE_TOPIC:
            pendulum_angle = str(msg.payload)
            msg = pendulum_angle.split(":")
            angle = msg[1].split(',')[0]
            speed = msg[2].split("'")[0]
            self_env.__insert_angle_info_to_buffer(device=Device.pendulum, angle=angle, theta=speed)
            #print("Pendulum Angle value: {0}, {1}".format(angle, speed))
        elif msg.topic == MQTT_MOTOR_ANGLE_TOPIC:
            motor_angle = str(msg.payload)
            msg = motor_angle.split(":")
            angle = msg[1].split(',')[0]
            speed = msg[2].split("'")[0]
            self_env.__insert_angle_info_to_buffer(device=Device.motor, angle=angle, theta=speed)
            #print("Motor Angle value: {0}, {1}".format(angle, speed))

    def __insert_angle_info_to_buffer(self, device, angle, theta):
        cosine_angle = math.cos(float(angle))
        sine_angle = math.sin(float(angle))
        time_epoch = datetime.now().timestamp()
        if device == Device.pendulum:
            self.pendulum_angle_buffer.append((time_epoch, [angle, cosine_angle, sine_angle, theta]))
        elif device == Device.motor:
            self.motor_angle_buffer.append((time_epoch, [angle, cosine_angle, sine_angle, theta]))
        else:
            raise AttributeError("Unrecognized Device")


    @staticmethod
    def __simulated_angle_generation(env):
        device_list = [Device.pendulum]
        # device_list = [Device.pendulum, Device.motor]
        while True:
            device = random.choice(device_list)
            angle = random.randrange(-100, 100)
            theta = random.randrange(-10, 10)
            env.__insert_angle_info_to_buffer(device=device, angle=angle, theta=theta)
            sleep_time = random.randrange(0, 10)
            time.sleep(sleep_time / 100.0)
            #print("insert - {0}, {1}, {2}".format(device, angle, theta))


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
        self.action_id = 0
        # t = threading.Thread(target=self.__simulated_angle_generation, args=(self,))
        # t.daemon = True
        # t.start()

        s = threading.Thread(target=self.__sub, args=(self.sub,))
        s.daemon = True
        s.start()

    def step(self, action):

        self.pub.publish(topic=MQTT_MOTOR_POWER_TOPIC, payload="speed:" + str(action))
        self.pub_to_EV3.publish(topic=MQTT_MOTOR_POWER_TOPIC, payload="speed:" + str(action))

        while True:
            if len(self.pendulum_angle_buffer) != 0 and len(self.motor_angle_buffer) != 0:
                break
            time.sleep(0.01)

        pendulum_angle = self.pendulum_angle_buffer.popleft()
        motor_angle = self.motor_angle_buffer.popleft()

        pendulum_angle_time_epoch = datetime.fromtimestamp(
            pendulum_angle[0]
        ).strftime('%Y-%m-%d %H:%M:%S')

        motor_angle_time_epoch = datetime.fromtimestamp(
            motor_angle[0]
        ).strftime('%Y-%m-%d %H:%M:%S')


        # print("Inside step: pendulum_angle_time_epoch - {0}, motor_angle_time_epoch - {1}". format(
        #     pendulum_angle_time_epoch,
        #     motor_angle_time_epoch
        # ))

        return pendulum_angle[1] + motor_angle[1], None, None

    def close(self):
        self.pub.publish(topic=MQTT_MOTOR_POWER_TOPIC, payload="speed:0")
        self.pub_to_EV3.publish(topic=MQTT_MOTOR_POWER_TOPIC, payload="speed:0")
