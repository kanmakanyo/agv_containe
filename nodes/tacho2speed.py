#!/usr/bin/env python3

import rospy
import time
import sys
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistStamped
from agv_container.msg import Nano_Tacho
from nav_msgs.msg import Odometry

temp_msg = {
    'v_fwr': 0.,
    'v_fwl': 0.,
    'v_rwr': 0.,
    'v_rwl': 0.
}

RUN = False
RUN_tacho = False

def tachoCallback(msg):
    global RUN_tacho

    pub_msg.header.seq = pub_msg.header.seq + 1
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.twist.angular.x = 0.5*np.pi/8/msg.FWR_pulse*10e6
    pub_msg.twist.angular.y = 0.5*np.pi/8/msg.FWL_pulse*10e6
    pub_msg.twist.angular.z = 0.5*np.pi/8/msg.RWR_pulse*10e6
    pub_msg.twist.linear.x = 0.5*np.pi/8/msg.RWL_pulse*10e6
    pub.publish(pub_msg)

    RUN_tacho = True



rospy.init_node('tacho2speed')
freq = 50 # Hz

pub_msg = TwistStamped()
pub_msg.header.frame_id = 'tacho2speed'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()


rospy.Subscriber('/tacho_data', Nano_Tacho, tachoCallback)
pub = rospy.Publisher('/tacho_speed', TwistStamped, queue_size=1)
rate = rospy.Rate(freq) # Hz


print("Waiting data from tacho...")
while not RUN_tacho:
    time.sleep(0.02) # 20 ms
    pass
print("Data from tacho received.")
print("tacho2speed program is now running")

rospy.spin()

