#!/usr/bin/env python

########## SIMPLE NAVIGATION LIVE PLOT
###### Written by DimasAP (github.com/dispectra)
###### init. code 2020-12-18

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import MultipleLocator
import rospy
import time
import sys
import os
from agv_container.msg import State_Estimator
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


########### INITIAL DATA
rospy.init_node('liveplot')
print("[INFO] rosnode 'liveplot' initialized")

waypoints_path = rospy.get_param('~waypoints_path', '291221_test10.npy')
# waypoints_path = '2021-02-16-12-20-13.npy'
waypoints_path = os.path.abspath(sys.path[0] + '/../src/waypoints/waypoints/' + waypoints_path)
print("waypoints file:", waypoints_path)
wp = np.load(waypoints_path)
# wp = wp[:10]
# wp[:,0] = wp[:,0]-60
# # wp[:,1] = wp[:,1]-60

x_front = np.min(wp[:,0])
y_front = np.min(wp[:,1])
x_rear = x_front + 1
y_rear = y_front + 1
yaw_gnss_fr = wp[0,2]

########### UPDATE DATA
def cbGnssFront(msg):
	global x_front
	global y_front

	x_front = msg.pose.pose.position.x
	y_front = msg.pose.pose.position.y

def cbGnssRear(msg):
	global x_rear
	global y_rear

	x_rear = msg.pose.pose.position.x
	y_rear = msg.pose.pose.position.y

def cbEstimator(msg):
	global yaw_gnss_fr
	yaw_gnss_fr = msg.yaw_gnss_fr

x_inst = x_front
y_inst = y_front

def cbConSig(msg):
	global x_inst
	global y_inst
	x_inst = msg.pose.covariance[6]
	y_inst = msg.pose.covariance[7]

rospy.Subscriber('/state_estimator', State_Estimator, cbEstimator)
rospy.Subscriber('/utm', Odometry, cbGnssFront)
rospy.Subscriber('/utm2', Odometry, cbGnssRear)
rospy.Subscriber('/control_signal', PoseWithCovarianceStamped, cbConSig)
print("[INFO] Subscribed to topics")

########### PLOT DATA
print("[INFO] Starting to plot..")
mpl.rcParams['toolbar'] = 'None'
fig = plt.figure(figsize=(3.5, 3.5))
fig.canvas.set_window_title('Navigation Live Plot')
ax = plt.gca()
ax.format_coord = lambda x, y: ''

def update_plot(i):
	# REF https://pythonprogramming.net/live-graphs-matplotlib-tutorial/
	global x_front
	global y_front
	global x_rear
	global y_rear
	global yaw_gnss_fr

	# plt.clf()
	ax.clear()

	## Waypoints
	# ax.plot(wp[:,0], wp[:,1], c="red", label=r"$(x,y)_{ref}$", lw=1, ls="--"))

	ax.scatter(wp[0,0], wp[0,1], c="red", label=r"$(x,y)_{ref0}$", s=5, zorder=2)
	ax.scatter(wp[:,0], wp[:,1], c="red", label=r"$(x,y)_{ref}$", s=0.5, zorder=2, alpha=0.2)
	
	## Instantaneous marker
	ax.scatter(x_front, y_front, marker="o", zorder=3, c="red", s=8, label="gnss_front")
	ax.scatter(x_rear, y_rear, marker="o", zorder=3, c="green", s=8, label="gnss_rear")
	ax.scatter(x_inst, y_inst, marker="x", zorder=3, c="blue", s=8, label="instant_ref")
	ax.plot([x_front, x_front+np.cos(yaw_gnss_fr)],
			[y_front, y_front+np.sin(yaw_gnss_fr)], c="blue", label="yaw_diff_fr")

	ax.legend(loc="upper right", fontsize=7)

	ax.set_xlim(np.min(wp[:,0])-2, np.max(wp[:,0])+2)
	ax.set_ylim(np.min(wp[:,1])-2, np.max(wp[:,1])+2)
	ax.axis('equal')
	ax.xaxis.set_minor_locator(MultipleLocator(1))
	ax.yaxis.set_minor_locator(MultipleLocator(1))
	ax.xaxis.set_major_locator(MultipleLocator(5))
	ax.yaxis.set_major_locator(MultipleLocator(5))
	ax.grid(color='gray', linestyle='--', linewidth=0.8, which='major', alpha=0.5)
	ax.grid(color='gray', linestyle='--', linewidth=0.5, which='minor', alpha=0.2)

	ax.tick_params(labelsize=6)
	ax.yaxis.offsetText.set_visible(False)
	ax.xaxis.offsetText.set_visible(False)
	_yofflabel = ax.yaxis.offsetText.get_text()
	_xofflabel = ax.xaxis.offsetText.get_text()
	ax.set_ylabel(r"$y_{utm}$ " + _yofflabel + " (m)", fontsize=7)
	ax.set_xlabel(r"$x_{utm}$ " + _xofflabel + " (m)", fontsize=7)	

live_plot = animation.FuncAnimation(fig, update_plot, interval=30)
plt.show()