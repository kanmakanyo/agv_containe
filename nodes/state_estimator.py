#!/usr/bin/env python3

import rospy
import time
import sys
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from agv_container.msg import State_Estimator
from nav_msgs.msg import Odometry

#-------------------------------------- MQTT --------------------------------------#
# import paho.mqtt.client as mqtt
# DEBUG_MQTT_SUB_MSG = False
# mqtt_gnss_fr_reverse = 0

# def on_mqtt_message(client, userdata, message):
#     global mqtt_gnss_fr_reverse

#     if DEBUG_MQTT_SUB_MSG:
#         print(rospy.Time.now(), "MQTT received", message.topic, (message.payload))
#     # print("Topic=", message.topic, "QoS", message.qos, "Retain", message.retain)

#     if message.topic == "state_estimator/gnss_fr_reverse":
#         mqtt_gnss_fr_reverse = int(message.payload)

# print("Creating new MQTT client ...")
# client = mqtt.Client("ros_state_estimator_node")
# client.on_message = on_mqtt_message

# print("Connecting to broker ...")
# broker_address = "localhost"
# client.connect(host="localhost", port=1883)

# print("Subscribing to topics ...")
# client.subscribe("state_estimator/gnss_fr_reverse")
#-----------------------------------------------------------------------------------#

sensor_data = {
    'x_front': 0.,
    'y_front': 0., 
    'x_rear': 0., 
    'y_rear': 0.
}

temp_msg = {
    'x_est': 0.,
    'y_est': 0.,
    'v_est': 0.,
    'yaw_est': 0.,
    'yaw_gnss_fr': 0.
}

RUN = False
RUN_gnss_front = False
RUN_gnss_rear = False

def gnssFrontCallback(msg):
    global sensor_data
    global temp_msg
    global RUN_gnss_front

    RUN_gnss_front = True


    sensor_data['x_front'] = msg.pose.pose.position.x
    sensor_data['y_front'] = msg.pose.pose.position.y

    delta_t = 0.2 #ms (5 Hz)
    temp_msg['v_est'] = np.sqrt((temp_msg['x_est']-sensor_data['x_front'])**2 + (temp_msg['y_est']-sensor_data['y_front'])**2) / delta_t
    temp_msg['x_est'] = sensor_data['x_front']
    temp_msg['y_est'] = sensor_data['y_front']
    temp_msg['yaw_gnss_fr'] = np.arctan2(sensor_data['y_front']-sensor_data['y_rear'],
                                                    sensor_data['x_front']-sensor_data['x_rear'])

    pub_msg.header.seq = pub_msg.header.seq + 1
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.yaw_gnss_fr = temp_msg['yaw_gnss_fr']
    pub_msg.x_est = temp_msg['x_est']
    pub_msg.y_est = temp_msg['y_est']
    pub_msg.v_est = temp_msg['v_est']
    pub.publish(pub_msg)
    

def gnssRearCallback(msg):
    global sensor_data
    global temp_msg
    global RUN_gnss_rear

    RUN_gnss_rear = True

    sensor_data['x_rear'] = msg.pose.pose.position.x
    sensor_data['y_rear'] = msg.pose.pose.position.y

    temp_msg['yaw_gnss_fr'] = np.arctan2(sensor_data['y_front']-sensor_data['y_rear'],
                                                    sensor_data['x_front']-sensor_data['x_rear'])

    pub_msg.header.seq = pub_msg.header.seq + 1
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.yaw_gnss_fr = temp_msg['yaw_gnss_fr']
    pub.publish(pub_msg)

rospy.init_node('estimator')
freq = 50 # Hz

pub_msg = State_Estimator()
pub_msg.header.frame_id = 'state_estimator'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()

rospy.Subscriber('/utm', Odometry, gnssFrontCallback)
rospy.Subscriber('/utm2', Odometry, gnssRearCallback)
pub = rospy.Publisher('/state_estimator', State_Estimator, queue_size=1)
rate = rospy.Rate(freq) # Hz


print("Waiting data from GNSS...")
while not RUN:
    RUN = RUN_gnss_front and RUN_gnss_rear
    time.sleep(0.02) # 20 ms
    pass
print("Data from GNSS received.")
print("State estimator program is now running")

rospy.spin()

