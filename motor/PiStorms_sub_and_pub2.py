import paho.mqtt.client as mqtt
import time
import datetime
import sys
import threading
from PiStorms import PiStorms

MQTT_SERVER = '10.0.0.1'

MQTT_MOTOR_RESET = 'reset'
MQTT_MOTOR_RECTIFY = 'rectify'
MQTT_MOTOR_POWER = 'motor_power'

MQTT_MOTOR_RESET_COMPLETE = 'reset_complete'
MQTT_MOTOR_RECTIFY_COMPLETE = 'rectify_complete'
MQTT_MOTOR_TO_PENDULUM = 'motor_angle'

sub_topic_list = [MQTT_MOTOR_RESET, MQTT_MOTOR_RECTIFY, MQTT_MOTOR_POWER]
pub_topic_list = [MQTT_MOTOR_RESET_COMPLETE, MQTT_MOTOR_RECTIFY_COMPLETE, MQTT_MOTOR_TO_PENDULUM]


class Motor:
    def __init__(self):
        self.psm = PiStorms()

        self.sub = mqtt.Client(client_id="motor_sub", transport="UDP")
        self.sub.on_connect = self.__on_connect
        self.sub.on_message = self.on_message
        self.sub.connect(MQTT_SERVER, 1883, 60)

        self.pub = mqtt.Client(client_id="motor_pub", transport="UDP")
        self.pub.connect(MQTT_SERVER, 1883, 60)

        self.speed = 0

    @staticmethod
    def __sub(sub, psm):
        try:
            print("Sub thread started!")
            sub.loop_forever()
        except KeyboardInterrupt:
            print("Sub Interrupted!")
            sub.unsubscribe(sub_topic_list)
            sub.disconnect()
            psm.BAM1.brake()

    def pub__(self, topic, payload):
        try:
            if topic in pub_topic_list:
                return self.pub.publish(topic=topic, payload=payload)
        except:
            print("The topic is not in pub_topic_list... retry...")
            sys.exit()

    @staticmethod
    def __on_connect(client, rc):
        print("mqtt broker connected with result code " + str(rc))
        client.subscribe(topic=sub_topic_list)

    def on_message(self, msg):
        if msg.topic == MQTT_MOTOR_RESET:
            if str(msg.payload) == MQTT_MOTOR_RESET:
                self.reset()

        if msg.topic == MQTT_MOTOR_RECTIFY:
            if str(msg.payload) == MQTT_MOTOR_RECTIFY:
                self.rectify()

        if msg.topic == MQTT_MOTOR_POWER:
            self.speed = str(msg.payload)
            self.psm.BAM1.setSpeed(self.speed)
            time.sleep(0.05)
            self.pub__(topic=MQTT_MOTOR_TO_PENDULUM, payload=self.get_angle())

    def start_sub_thread(self):
        s = threading.Thread(target=self.__sub, args=(self.sub, self.psm))
        s.daemon = True
        s.start()

    def reset(self):
        isError = True
        # reset position
        self.psm.BAM1.brake()

        while isError:
            try:
                angle = self.psm.BAM1.pos()
                isError = False if angle < 600 and angle > -600 else True
            except TypeError as e:
                isError = True
            time.sleep(0.001)

        self.psm.BAM1.runDegs(-angle, 30, True, True)
        time.sleep(3)
        self.pub__(topic=MQTT_MOTOR_RESET_COMPLETE, payload=MQTT_MOTOR_RESET_COMPLETE)

    def rectify(self):
        self.psm.BAM1.brake()
        cnt_sleep = 0
        while cnt_sleep < 8:
            print ".",
            sys.stdout.flush()
            time.sleep(1)
            cnt_sleep += 1
        print()

        self.psm.BAM1.setSpeed(100)
        time.sleep(0.3)

        self.psm.BAM1.setSpeed(0)
        time.sleep(0.85)

        self.psm.BAM1.setSpeed(100)
        time.sleep(0.2)

        self.psm.BAM1.setSpeed(-100)
        time.sleep(0.2)

        # self.psm.BAM1.setSpeed(0)
        # time.sleep(0.05)
        self.pub__(topic=MQTT_MOTOR_RECTIFY_COMPLETE, payload=self.get_angle())

    def get_angle(self):
        isError = True
        while isError:
            try:
                angle = self.psm.BAM1.pos()
                isError = False if angle < 600 and angle > -600 else True
                if angle > 360 or angle < -360 and isError == False:
                    self.psm.BAM1.brake()
            except TypeError as e:
                isError = True
        return angle


if __name__ == "__main__":
    motor = Motor()
    motor.start_sub_thread()

