import paho.mqtt.client as mqtt
import time
import datetime
from PiStorms import PiStorms
psm = PiStorms()
speed = 0

MQTT_SERVER = '10.0.0.1'
MQTT_MOTOR_POWER_TOPIC = 'Motor/power'
MQTT_MOTOR_ANGLE_TOPIC = 'Motor/angle'

pub = mqtt.Client(client_id="motor_angle_pub", transport="UDP")
pub.connect(MQTT_SERVER, 1883, 60)

def on_connect(client, userdata, flags, rc) :
    print("connected with result code " + str(rc))
    client.subscribe(MQTT_MOTOR_POWER_TOPIC)

def on_message(client, useradta, msg) :
    global speed
    last_angle = 0
    # print(msg.payload)
    speed = int(str(msg.payload).split(":")[1])
    if speed == 999999:        # reset position
        try:
            psm.BAM1.brake()
            time.sleep(0.3)
            angle = psm.BAM1.pos()
            last_angle = angle
        except TypeError as e:
            angle = last_angle
        psm.BAM1.runDegs(-angle, 50, True, True)
	time.sleep(3)
	psm.BAM1.resetPos()
    psm.BAM1.setSpeed(speed)
    time.sleep(0.01)
    try:
        angle = psm.BAM1.pos()
        last_angle = angle
    except TypeError as e:
        angle = last_angle
    pub.publish(topic=MQTT_MOTOR_ANGLE_TOPIC, payload="angle:" + str(angle))
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
    psm.BAM1.brake()

