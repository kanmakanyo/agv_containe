#!/usr/bin/env python3

import rospy
import time
import sys
import os
import numpy as np
# from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from agv_container.msg import State_Estimator
# import paho.mqtt.client as mqtt
# sys.path.append(os.path.abspath(sys.path[0] + '/../src/python'))
# from stanley_2d import Controller
# # from lyapunov_rl import Controller

#---------------------- MQTT -------------------------------#
# DEBUG_MQTT_SUB_MSG = False
# mqtt_autonomous = 0
# mqtt_autonomous_bef = mqtt_autonomous
# mqtt_steer = 0
# mqtt_throttle = 0
# mqtt_brake = 0
# mqtt_locker = 0
# mqtt_speed_control_mode = 0
# mqtt_speed_setpoint = 0
# mqtt_brake_regen = 0

# def on_mqtt_message(client, userdata, message):
#     global mqtt_autonomous
#     global mqtt_steer
#     global mqtt_throttle
#     global mqtt_brake
#     global mqtt_locker
#     global mqtt_brake_regen

#     if DEBUG_MQTT_SUB_MSG:
#         print(rospy.Time.now(), "MQTT received", message.topic, (message.payload))
#     # print("Topic=", message.topic, "QoS", message.qos, "Retain", message.retain)

#     if message.topic == "control/autonomous":
#         mqtt_autonomous = int(message.payload)
#     if message.topic == "control/steer":
#         mqtt_steer = float(message.payload)
#     if message.topic == "control/throttle":
#         mqtt_throttle = float(message.payload)
#     if message.topic == "control/brake":
#         mqtt_brake = float(message.payload)
#     if message.topic == "control/locker":
#         mqtt_locker = int(message.payload)
#     if message.topic == "control/brake_regen":
#         mqtt_brake_regen = float(message.payload)

#-------------------------------------------------------------#

state = {'x': 0., 'y': 0., 'yaw': 0., 'v': 0.}

def callbackStateEstimator(msg):
    global state
    global RECEIVED_STATE_VAL
    state['x'] = msg.x_est
    state['y'] = msg.y_est
    state['yaw'] = msg.yaw_gnss_fr
    state['v'] = msg.v_est
    RECEIVED_STATE_VAL = True


#------------------- Ultrasonic ---------------------#
ULTRASONIC_STOP_DISTANCE = 200 # in centimeter
interlock_ultrasonic = False

def cbNanoUltrasonicFront(msg):
	global interlock_ultrasonic

    if (msg.twist.linear.x < ULTRASONIC_STOP_DISTANCE):
        interlock_ultrasonic = True
    if (msg.twist.linear.y < ULTRASONIC_STOP_DISTANCE):
        interlock_ultrasonic = True
    if (msg.twist.linear.z < ULTRASONIC_STOP_DISTANCE):
        interlock_ultrasonic = True
    if (msg.twist.angular.x < ULTRASONIC_STOP_DISTANCE):
        interlock_ultrasonic = True
    if (msg.twist.angular.y < ULTRASONIC_STOP_DISTANCE):
        interlock_ultrasonic = True

def cbNanoUltrasonicRear(msg):
	global interlock_ultrasonic

    if (msg.twist.linear.x < ULTRASONIC_STOP_DISTANCE):
        interlock_ultrasonic = True
    if (msg.twist.linear.y < ULTRASONIC_STOP_DISTANCE):
        interlock_ultrasonic = True
    if (msg.twist.linear.z < ULTRASONIC_STOP_DISTANCE):
        interlock_ultrasonic = True
    if (msg.twist.angular.x < ULTRASONIC_STOP_DISTANCE):
        interlock_ultrasonic = True
    if (msg.twist.angular.y < ULTRASONIC_STOP_DISTANCE):
        interlock_ultrasonic = True
#-------------------------------------------------------------#

#--------------------------- GNSS ----------------------------#
interlock_gnss = False

#-------------------------------------------------------------#


rospy.init_node('safety_interlock')
freq = 50 # Hz

rospy.Subscriber('/state_estimator', State_Estimator, callbackStateEstimator)
rospy.Subscriber('/control_signal', PoseWithCovarianceStamped, callbackConSig)
rospy.Subscriber('/nano_ultrasonic_front', TwistStamped, cbNanoUltrasonicFront)
rospy.Subscriber('/nano_ultrasonic_rear', TwistStamped, cbNanoUltrasonicRear)
rospy.Subscriber('/utm', Odometry, cbGnssFront)
rospy.Subscriber('/utm2', Odometry, cbGnssRear)

pub = rospy.Publisher('/safety_interlock', PoseWithCovarianceStamped, queue_size=1)

freq = rospy.get_param('~freq', 20.) # Hz
ff_1 = rospy.get_param('~ff_1', 0.0)
ff_2 = rospy.get_param('~ff_2', 0.0)
kp = rospy.get_param('~kp', 0.11)
ki = rospy.get_param('~ki', 0.30)
kd = rospy.get_param('~kd', 0.015)
sat_long_max = rospy.get_param('~sat_long_max', 0.3)
sat_long_min = rospy.get_param('~sat_long_min', -2.9)
kv_yaw = rospy.get_param('~kv_yaw', 2.25)
kv_lat = rospy.get_param('~kv_lat', 0.75)
min_vel_move = rospy.get_param('~min_vel_move', 0.5)
max_throttle_move = rospy.get_param('~max_throttle_move', 0.3)
min_throttle_move = rospy.get_param('~min_throttle_move', 0.3)
length = rospy.get_param('~length', 1.7)
ks = rospy.get_param('~ks', 0.75)
kv = rospy.get_param('~kv', 1.00)
lateral_dead_band = rospy.get_param('~lateral_dead_band', 0.025)
sat_lat_max = rospy.get_param('~sat_lat_max', 0.6109)
sat_lat_min = rospy.get_param('~sat_lat_min', -0.4887)
waypoints_path = rospy.get_param('~waypoints_path', 'wp_monev_baru.npy')
waypoints_path = os.path.abspath(sys.path[0] + '/../src/waypoints/waypoints/' + waypoints_path)


feed_forward_params = np.array([ff_1, ff_2])
sat_long = np.array([sat_long_min, sat_long_max])
sat_lat = np.array([sat_lat_min, sat_lat_max])
waypoints = np.load(waypoints_path)
# TEMP: override v setpoint
for i in range(len(waypoints)):
    waypoints[i,3] = 1 # speed in m/s

#-------------- MQTT Client setup -------------#
# print("Creating new MQTT client ...")
# client = mqtt.Client("safety_interlock")
# client.on_message = on_mqtt_message

# print("Connecting to broker ...")
# broker_address = "localhost"
# broker_port = 1883
# client.connect(host=broker_address, port=broker_port)

# print("Subscribing to topics ...")
# client.subscribe("control/autonomous")
# client.subscribe("control/steer")
# client.subscribe("control/throttle")
# client.subscribe("control/brake")
# client.subscribe("control/locker")

# topic = "test/publish/topic"
# print("Publishing message to topic",topic)
# client.publish(topic,"OFF")

# client.loop_start()
#-----------------------------------------------#


rate = rospy.Rate(freq) # Hz
pub_msg = PoseWithCovarianceStamped()
pub_msg.header.frame_id = 'safety_interlock_msg'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()
last_time = pub_msg.header.stamp.to_sec() - 1./freq
last_time = rospy.Time.now().to_sec() - 1./freq

while not rospy.is_shutdown():
    ### Calculate the actual sampling time
    pub_msg.header.stamp = rospy.Time.now()
    delta_t = pub_msg.header.stamp.to_sec() - last_time
    last_time = pub_msg.header.stamp.to_sec()

    ### Send the message
    # Header
    pub_msg.header.seq += 1

    # Assign message data
    pub_msg.pose.covariance[0] = interlock_ultrasonic
    pub_msg.pose.covariance[1] = interlock_gnss

    # Publish the message
    pub.publish(pub_msg)

    ### Wait until the next loop
    rate.sleep()