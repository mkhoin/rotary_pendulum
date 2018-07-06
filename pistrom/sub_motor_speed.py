import paho.mqtt.client as mqtt
import time
import datetime
from PiStorms import PiStorms
psm = PiStorms()
speed = 0

MQTT_SERVER = '10.0.0.1'
MQTT_MOTOR_POWER_TOPIC = 'Motor/power'

def on_connect(client, userdata, flags, rc) :
    print("connected with result code " + str(rc))
    client.subscribe(MQTT_MOTOR_POWER_TOPIC)

def on_message(client, useradta, msg) :
    global speed
    #print(msg.payload)
    speed = int(str(msg.payload).split(":")[1])
    psm.BAM1.setSpeed(speed)
    #psm.BAM1.floatSync()

sub = mqtt.Client()
sub.on_connect = on_connect
sub.on_message = on_message

sub.connect(MQTT_SERVER, 1883, 60)

try :
    sub.loop_forever()
except KeyboardInterrupt :
    sub.unsubscribe(["Motor/speed"])
    sub.disconnect()
#    psm.BAM1.setSpeed(0)
