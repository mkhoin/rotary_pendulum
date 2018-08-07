import paho.mqtt.client as mqtt
import time
import datetime
import ev3dev.ev3 as ev3
import sys
from smbus import SMBus
import threading
import ctypes

speed = 0

MQTT_SERVER = '192.168.137.4'

MQTT_MOTOR_AND_PENDULUM_STATE = 'state_info'
MQTT_MOTOR_RECTIFY_COMPLETE = 'rectify_complete'
MQTT_MOTOR_TO_PENDULUM = 'motor_angle'

sub_topic_list = [MQTT_MOTOR_RECTIFY_COMPLETE, MQTT_MOTOR_TO_PENDULUM]
pub_topic_list = [MQTT_MOTOR_AND_PENDULUM_STATE]

MODE_NORMAL = 0x00
MODE_CALIBRATE = 0x43
MODE_RESET = 0x52


class Pendulum:
    def __init__(self):
        self.hta = HTAngle(1, 0x01)
        while True:
            try:
                port = ev3.LegoPort("pistorms:BBS2")
                if port.mode != "i2c-thru":
                    port.mode = "i2c-thru"
                break
            except Exception as e:
                print("I/O exception.")
                time.sleep(0.1)
        self.hta.reset()

        self.sub = mqtt.Client(client_id="pendulum_sub", transport="UDP")
        self.sub.on_connect = self.__on_connect
        self.sub.on_message = self.on_message
        self.sub.connect(MQTT_SERVER, 1883, 60)

        self.pub = mqtt.Client(client_id="pendulum_pub", transport="UDP")
        self.pub.connect(MQTT_SERVER, 1883, 60)

        self.speed = 0

    @staticmethod
    def __sub(sub):
        try:
            print("Sub thread started!")
            sub.loop_forever()
        except KeyboardInterrupt:
            print("Sub Interrupted!")
            sub.unsubscribe(sub_topic_list)
            sub.disconnect()

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
        if msg.topic in sub_topic_list:
            motor_angle = str(msg.payload.decode("utf-8"))
            pendulum_angle, pendulum_speed = self.get_pendulum_state()
            self.pub__(topic=MQTT_MOTOR_AND_PENDULUM_STATE, payload=pendulum_angle+'|'+pendulum_speed+'|'+motor_angle)

    def start_sub_thread(self):
        s = threading.Thread(target=self.__sub, args=(self.sub, self.psm))
        s.daemon = True
        s.start()

    def get_pendulum_state(self):
        info = True
        while info:
            ht, info = self.hta.update()  # [angle, acc_angle, rpm], isError
            time.sleep(0.001)
        return (str(ht[0] - 180), str(ht[2]))


class HTAngle:
    def __init__(self, bus_no, address):
        self.bus = SMBus(bus_no)
        self.address = address

        self.angle = 0              # angle, degrees(0 - 359)
        self.acc_angle = 0          # accumulated angle, degrees(-2147483648 - 2147483647)
        self.rpm = 0                # rotations per minute (-1000 to 1000)

        self.update()

    def set_mode(self, mode):
        self.bus.write_byte_data(self.address, 0x41, mode)

    def get_mode(self):
        return self.bus.read_byte_data(self.address, 0x41)

    def reset(self):
        # reset accumulated angle
        self.set_mode(MODE_RESET)
        time.sleep(0.1)

        # calibrate angle
        self.set_mode(MODE_CALIBRATE)
        time.sleep(0.1)

        self.update()

    def update(self):
        isError = True
        while isError:
            try:
                data = self.bus.read_i2c_block_data(self.address, 0x41, 10)
                isError = False
            except OSError as e:
                print(e)
                isError = True

        # error filtering
        if data[1] == data[2] == data[3] == data[4] == data[5] == data[6] == data[7] == data[8] == 0xff:
            return (self.angle, self.acc_angle, self.rpm), True

        # angle
        self.angle = (data[1] & 0xff) * 2 + (data[2] & 0x01) + ((data[2] & 0x80) << 1)

        # accumulated angle
        acc = (((data[3] & 0x7f) + (data[4] & 0x80)) * 0x1000000)\
              + (((data[4] & 0x7f) + (data[5] & 0x80)) * 0x10000)\
              + (((data[5] & 0x7f) + (data[6] & 0x80)) * 0x100)\
              + ((data[6] & 0x7f) + (data[7] & 0x80))

        # convert to signed
        if acc > 0x7fffffff:
            self.acc_angle = (0x100000000 - acc) * (-1)
        else:
            self.acc_angle = acc

        # rpm
        rpm = (data[7] & 0x7f) * 0x100 + (data[8] & 0x7f) + (data[9] & 0x80)

        # convert to signed
        if rpm > 0x3fff:
            self.rpm = (0x8000 - rpm) * (-1)
        else:
            self.rpm = rpm
        return (self.angle, self.acc_angle, self.rpm), False


if __name__ == "__main__":
    pendulum = Pendulum()
    pendulum.start_sub_thread()