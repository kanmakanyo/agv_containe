#!/usr/bin/env python3

import rospy
import time
import sys
# from agv_container.msg import Mega_Container
from agv_container.msg import State_Estimator
# from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

import paho.mqtt.client as mqtt #import the client1

# def on_mqtt_message(client, userdata, message):
#     global mqtt_steer
#     global pub_msg

#     print("MQTT received", message.topic, float(message.payload))
#     # print("Topic=", message.topic, "QoS", message.qos, "Retain", message.retain)
#     if message.topic == "control/steer": 
#         mqtt_steer = float(message.payload) 
#         # pub_msg.linear.x = mqtt_steer
#     if message.topic == "control/throttle": 
#         mqtt_throttle = float(message.payload)
#         # pub_msg.linear.y = mqtt_throttle
#     if message.topic == "control/brake": 
#         mqtt_brake = float(message.payload)
#         # pub_msg.linear.z = mqtt_brake
#     if message.topic == "control/locker": 
#         mqtt_locker = int(message.payload)

print("Creating new MQTT client ...")
client = mqtt.Client("ros2mqtt") #create new instance
# client.on_message = on_mqtt_message   

print("Connecting to broker ...")
broker_address = "localhost"
client.connect(host=broker_address, port=1883)

# print("Subscribing to topics ...")
# client.subscribe("control/steer")
# client.subscribe("control/throttle")

client.loop_start()


def callbackLogArduino(msg):
    client.publish("/logging_arduino/steer_front_actual_rad", msg.pose.covariance[7])
    # print('Published from ROS topic /logging_arduino to MQTT')

def callbackStateEstimator(msg):
    client.publish("/state_estimator/v_est", msg.v_est)
    # print('Published from ROS topic /state_estimator to MQTT')

# while not rospy.is_shutdown():
#     ### Calculate the actual sampling time
#     # msg.header.stamp = rospy.Time.now()
#     # delta_t = msg.header.stamp.to_sec() - last_time
#     # last_time = msg.header.stamp.to_sec()

#     # t_now = rospy.Time.now()
#     # delta_t = t_now.to_sec() - last_time
#     # last_time = t_now.to_sec()

#     ### Calculate the control signal

#     ### Send the message
#     # Header
#     # pub_msg.header.seq += 1
    
#     # Control action
#     # pub_msg.linear.x = mqtt_steer
#     # pub_msg.linear.y = mqtt_throttle
#     # pub_msg.linear.z = mqtt_brake
#     # pub_msg.angular.x = mqtt_locker
    
#     # Publish the message
#     pub.publish(pub_msg)

#     ### Wait until the next loop
#     rate.sleep()

if __name__ == '__main__':
    rospy.init_node('ros2mqtt')
    rospy.Subscriber('/logging_arduino', PoseWithCovarianceStamped, callbackLogArduino)
    rospy.Subscriber('/state_estimator', State_Estimator, callbackStateEstimator)
    rospy.spin()

