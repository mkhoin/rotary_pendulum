import paho.mqtt.client as mqtt
import time
import datetime
import sys

import time
import random

from PiStorms import PiStorms

psm = PiStorms()

MQTT_SERVER="10.0.0.1"
MQTT_MOTOR_ANGLE_TOPIC = 'Motor/angle'

pub = mqtt.Client(client_id="motor_angle_pub", transport="UDP")
pub.connect(MQTT_SERVER, 1883, 60)

doExit = False
old_pos = 0
encoder_pos = -1

i = 0
while True:
    now = datetime.datetime.now()
    
    last_angle = 0
    try:
        angle = psm.BAM1.pos()
        #speed = (angle - last_angle) / 0.05
        speed = (angle - last_angle) / 5
        last_angle = angle
    except TypeError as e:
        angle = -100000
        speed = -100000
        
    pub.publish(topic=MQTT_MOTOR_ANGLE_TOPIC, payload="angle:" + str(angle) + ", speed:" + str(speed))
   # time.sleep(0.05)
    time.sleep(5)
    psm.BAM1.floatSync()
    #print i,
    #i += 1
    #sys.stdout.flush()

