#https://github.com/ev3dev/ev3dev-lang-python/blob/jessie/ev3dev/core.py
import paho.mqtt.client as mqtt
import time
import ev3dev.ev3 as ev3
import datetime
import sys

MQTT_SERVER = "192.168.137.4"
MQTT_PENDULUM_ANGLE_TOPIC = 'Pendulum/angle'

pub = mqtt.Client(client_id="pendulum_angle_pub", transport="UDP")
pub.connect(MQTT_SERVER, 1883, 60)

while(1):
    i = 0
    ev3.Sensor(ev3.INPUT_1).mode = 'ANGLE'          # sensor mode = angle
    sensor_angle = ev3.Sensor(ev3.INPUT_1).value()  # angle of pendulum
    ev3.Sensor(ev3.INPUT_1).mode = 'SPEED'          # sensor mode = speed
    sensor_speed = ev3.Sensor(ev3.INPUT_1).value()  # speed of pendulum

    pub.publish(MQTT_PENDULUM_ANGLE_TOPIC, payload="angle:" + str(sensor_angle) + ", speed:" + str(sensor_speed))
    time.sleep(0.05)
    #print(i, end=" ")
    #i += 1
    #sys.stdout.flush()

