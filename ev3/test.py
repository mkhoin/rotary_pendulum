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

sensor = ev3.Sensor(ev3.INPUT_1)
#sensor.command = 'RESET'
#sensor.command = 'CAL'

print(sensor.num_values)

print()

sensor.mode = 'ANGLE'          # sensor mode = angle
sensor_angle = sensor.value()  # angle of pendulum
print(sensor_angle)
print(sensor.units)

print()

sensor.mode = 'SPEED'          # sensor mode = speed
sensor_speed = sensor.value()  # speed of pendulum
print(sensor_speed)
print(sensor.units)

print(sensor.commands)
