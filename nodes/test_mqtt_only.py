import paho.mqtt.client as mqtt
# import rospy

index = 1


#---------------------- MQTT -------------------------------#
DEBUG_MQTT_SUB_MSG = True
mqtt_autonomous = 0
mqtt_autonomous_bef = mqtt_autonomous
mqtt_steer = 0
mqtt_throttle = 0
mqtt_brake = 0
mqtt_locker = 0
mqtt_speed_control_mode = 0
mqtt_speed_setpoint = 0
mqtt_brake_regen = 0

def on_mqtt_message(client, userdata, message):
    global mqtt_autonomous
    global mqtt_steer
    global mqtt_throttle
    global mqtt_brake
    global mqtt_locker
    global mqtt_brake_regen
    global index

    if DEBUG_MQTT_SUB_MSG:
        # print(rospy.Time.now(), "MQTT received", message.topic, (message.payload))
        print("Topic=", message.topic, "QoS", message.qos, "Retain", message.retain)
        print(index)
        index +=1

    if message.topic == "control/autonomous":
        mqtt_autonomous = int(message.payload)
    if message.topic == "control/steer":
        mqtt_steer = float(message.payload)
    if message.topic == "control/throttle":
        mqtt_throttle = float(message.payload)
    if message.topic == "control/brake":
        mqtt_brake = float(message.payload)
    if message.topic == "control/locker":
        mqtt_locker = int(message.payload)
    if message.topic == "control/brake_regen":
        mqtt_brake_regen = float(message.payload)



#-------------- MQTT Client setup -------------#
print("Creating new MQTT client ...")
client = mqtt.Client("ros_control_node")
client.on_message = on_mqtt_message

print("Connecting to broker ...")
broker_address = "127.0.0.1"
client.connect(host=broker_address, port=1883)

print("Subscribing to topics ...")
client.subscribe("control/autonomous")
client.subscribe("control/steer")
client.subscribe("control/throttle")
client.subscribe("control/brake")
client.subscribe("control/locker")

# topic = "test/publish/topic"
# print("Publishing message to topic",topic)
# client.publish(topic,"OFF")

client.loop_start()
#--------------------

while 1:
    pass