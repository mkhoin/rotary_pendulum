#https://github.com/ev3dev/ev3dev-lang-python/blob/jessie/ev3dev/core.py
import paho.mqtt.client as mqtt
import time
import ev3dev.ev3 as ev3
import datetime
import sys

MQTT_SERVER = "192.168.137.4"
#MQTT_SERVER_PI = "10.0.0.1"
MQTT_PENDULUM_ANGLE_TOPIC = 'Pendulum/angle'
MQTT_MOTOR_POWER_TOPIC = 'Motor/power'

pub = mqtt.Client(client_id="pendulum_angle_pub", transport="UDP")
pub.connect(MQTT_SERVER, 1883, 60)


def on_connect(client, userdata, flags, rc) :
    print("connected with result code " + str(rc))
    client.subscribe(MQTT_MOTOR_POWER_TOPIC)

def on_message(client, useradta, msg) :
    global speed
    #print(msg.payload)
    speed = int(str(msg.payload).split(":")[1])
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

sub = mqtt.Client()
sub.on_connect = on_connect
sub.on_message = on_message

sub.connect(MQTT_SERVER, 1883, 60)

try :
    sub.loop_forever()
except KeyboardInterrupt :
    sub.unsubscribe(["Motor/power"])
    sub.disconnect()


