#!/usr/bin/env python

########## SIMPLE NAVIGATION LIVE PLOT
###### Written by DimasAP (github.com/dispectra)
###### init. code 2020-12-18

from ctypes import c_char
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import MultipleLocator
from shapely.geometry import Polygon
import rospy
import time
import sys
import os
from agv_container.msg import State_Estimator
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
# from geometry_msgs.msg import PoseWithCovarianceStamped


########### INITIAL DATA
rospy.init_node('liveplot')
print("[INFO] rosnode 'liveplot' initialized")

# waypoints_path = rospy.get_param('~waypoints_path', 'wp_monev_baru.npy')
# waypoints_path = '2021-02-16-12-20-13.npy'
# waypoints_path = os.path.abspath(sys.path[0] + '/../src/waypoints/waypoints/' + waypoints_path)
# print("waypoints file:", waypoints_path)
# wp = np.load(waypoints_path)
# wp = wp[:10]
# wp[:,0] = wp[:,0]-60
# # wp[:,1] = wp[:,1]-60

def plot_edge_car(x,y,yaw,rot_x,rot_y):

    x1 = x + rot_x*np.cos(yaw) - rot_y*np.sin(yaw)
    y1 = y + rot_x*np.sin(yaw) + rot_y*np.cos(yaw)

    x2 = x - rot_x*np.cos(yaw) - rot_y*np.sin(yaw)
    y2 = y - rot_x*np.sin(yaw) + rot_y*np.cos(yaw)

    x3 = x - rot_x*np.cos(yaw) + rot_y*np.sin(yaw)
    y3 = y - rot_x*np.sin(yaw) - rot_y*np.cos(yaw)

    x4 = x + rot_x*np.cos(yaw) + rot_y*np.sin(yaw)
    y4 = y + rot_x*np.sin(yaw) - rot_y*np.cos(yaw)

    x5 = x + (rot_x+rot_x/5.9)*np.cos(yaw) + (rot_y-rot_y/1.9)*np.sin(yaw)
    y5 = y + (rot_x+rot_x/5.9)*np.sin(yaw) - (rot_y-rot_y/1.9)*np.cos(yaw)

    x6 = x + (rot_x+rot_x/5.9)*np.cos(yaw) - (rot_y-rot_y/1.9)*np.sin(yaw)
    y6 = y + (rot_x+rot_x/5.9)*np.sin(yaw) + (rot_y-rot_y/1.9)*np.cos(yaw)

    xy = [[x1,y1],[x2,y2],[x3,y3],[x4,y4],[x5,y5],[x6,y6],[x1,y1]]

    return xy

def plot_edge_wheel(x,y,yaw,rot_x,rot_y):
    
    x1 = x + rot_x*np.cos(yaw) - rot_y*np.sin(yaw)
    y1 = y + rot_x*np.sin(yaw) + rot_y*np.cos(yaw)

    x2 = x - rot_x*np.cos(yaw) - rot_y*np.sin(yaw)
    y2 = y - rot_x*np.sin(yaw) + rot_y*np.cos(yaw)

    x3 = x - rot_x*np.cos(yaw) + rot_y*np.sin(yaw)
    y3 = y - rot_x*np.sin(yaw) - rot_y*np.cos(yaw)

    x4 = x + rot_x*np.cos(yaw) + rot_y*np.sin(yaw)
    y4 = y + rot_x*np.sin(yaw) - rot_y*np.cos(yaw)

    xy = [[x1,y1],[x2,y2],[x3,y3],[x4,y4],[x1,y1]]

    return xy

x_front = 0
y_front = 0
x_rear = x_front + 1
y_rear = y_front + 1
yaw_gnss_fr = 0
steer = 0

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

def cbEncoder(msg):
    global steer

    steer = msg.pose.covariance[7]

# x_inst = x_front
# y_inst = y_front

# def cbConSig(msg):
# 	global x_inst
# 	global y_inst
# 	x_inst = msg.pose.covariance[6]
# 	y_inst = msg.pose.covariance[7]

rospy.Subscriber('/state_estimator', State_Estimator, cbEstimator)
rospy.Subscriber('/utm', Odometry, cbGnssFront)
rospy.Subscriber('/utm2', Odometry, cbGnssRear)
rospy.Subscriber('/logging_arduino', PoseWithCovarianceStamped, cbEncoder)
# rospy.Subscriber('/control_signal', PoseWithCovarianceStamped, cbConSig)
print("[INFO] Subscribed to topics")

########### PLOT DATA
print("[INFO] Starting to plot..")
mpl.rcParams['toolbar'] = 'None'
fig = plt.figure(figsize=(15, 8))
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
	global steer

	xmin_lim = 788365.9561 - 10
	xmax_lim = 788365.9561 + 10
	ymin_lim = 9237580.6824 - 10
	ymax_lim = 9237580.6824 + 10
	rot_c_x = 2
	rot_c_y = 0.6

	rot_w_x = 1.2
	rot_w_y = 0.4

	rot_x = 0.3
	rot_y = 0.5

	xr = x_rear + rot_x*np.cos(yaw_gnss_fr) + rot_y*np.sin(yaw_gnss_fr)
	yr = y_rear + rot_x*np.sin(yaw_gnss_fr) - rot_y*np.cos(yaw_gnss_fr)

	xc = xr + rot_w_x*np.cos(yaw_gnss_fr)
	yc = yr + rot_w_x*np.sin(yaw_gnss_fr)

	edge_car = plot_edge_car(xc, yc, yaw_gnss_fr, rot_c_x, rot_c_y)

	pos_wheel = np.array(plot_edge_wheel(xc, yc, yaw_gnss_fr, rot_w_x, rot_w_y))

	edge_wheel_1 = plot_edge_wheel(pos_wheel[0,0], pos_wheel[0,1], yaw_gnss_fr+steer, 0.25, 0.12)
	edge_wheel_2 = plot_edge_wheel(pos_wheel[1,0], pos_wheel[1,1], yaw_gnss_fr, 0.25, 0.12)
	edge_wheel_3 = plot_edge_wheel(pos_wheel[2,0], pos_wheel[2,1], yaw_gnss_fr, 0.25, 0.12)
	edge_wheel_4 = plot_edge_wheel(pos_wheel[3,0], pos_wheel[3,1], yaw_gnss_fr+steer, 0.25, 0.12)

	x_c,y_c = Polygon(edge_car).exterior.xy
	x_w1,y_w1 = Polygon(edge_wheel_1).exterior.xy
	x_w2,y_w2 = Polygon(edge_wheel_2).exterior.xy
	x_w3,y_w3 = Polygon(edge_wheel_3).exterior.xy
	x_w4,y_w4 = Polygon(edge_wheel_4).exterior.xy

	xlim = [xmin_lim, xmax_lim, xmax_lim, xmin_lim]
	ylim = [ymin_lim, ymin_lim, ymax_lim, ymax_lim]
	
	# ax.scatter(xlim, ylim)    

	#xp1,yp1 = Polygon([[xmin_lim, ymin_lim], [xmax_lim, ymin_lim], [xmax_lim, ymax_lim], [xmin_lim, ymax_lim], [xmin_lim, ymin_lim]]).exterior.xy

	# ax.clf()
	ax.clear()

	## Waypoints
	# ax.plot(wp[:,0], wp[:,1], c="red", label=r"$(x,y)_{ref}$", lw=1, ls="--"))
	# ax.scatter(wp[0,0], wp[0,1], c="red", label=r"$(x,y)_{ref0}$", s=5, zorder=2)
	# ax.scatter(wp[:,0], wp[:,1], c="red", label=r"$(x,y)_{ref}$", s=0.5, zorder=2, alpha=0.2)

	ax.scatter(xlim, ylim)    
	## Instantaneous marker
	ax.scatter(x_front, y_front, marker="o", c="red", label="gnss_front")
	ax.scatter(x_rear, y_rear, marker="o", zorder=3, c="green", label="gnss_rear")
	# ax.scatter(x_inst, y_inst, marker="x", zorder=3, c="blue", s=8, label="instant_ref")
	ax.plot([x_front, x_rear],[y_front, y_rear], c="blue", label="yaw_diff_fr")
	ax.plot(xr,yr, marker="o", zorder=3, c="black", label="Rear wheel")	
	ax.plot(xc,yc, marker="o", zorder=3, c="blue", label="Car Position")	
	# ax.plot(xp1,yp1, "k")
	ax.plot(x_c,y_c,"k")
	ax.plot(x_w1,y_w1,"b")
	ax.plot(x_w2,y_w2,"b")
	ax.plot(x_w3,y_w3,"b")
	ax.plot(x_w4,y_w4,"b")

	# ax.clear()

	ax.legend(loc="upper right", fontsize=7)
	# ax.clear()

	# ax.set_xlim(np.min(wp[:,0])-2, np.max(wp[:,0])+2)
	# ax.set_ylim(np.min(wp[:,1])-2, np.max(wp[:,1])+2)
	# ax.set_xlim(xmin_lim, xmax_lim)
	# ax.set_ylim(ymin_lim, ymax_lim)
	# ax.axis("equal")
#	ax.xaxis.set_minor_locator(MultipleLocator(1))
#	ax.yaxis.set_minor_locator(MultipleLocator(1))
#	ax.xaxis.set_major_locator(MultipleLocator(5))
#	ax.yaxis.set_major_locator(MultipleLocator(5))
#	ax.grid(color='gray', linestyle='--', linewidth=0.8, which='major', alpha=0.5)
#	ax.grid(color='gray', linestyle='--', linewidth=0.5, which='minor', alpha=0.2)

#	ax.tick_params(labelsize=6)
#	ax.yaxis.offsetText.set_visible(False)
#	ax.xaxis.offsetText.set_visible(False)
#	_yofflabel = ax.yaxis.offsetText.get_text()
#	_xofflabel = ax.xaxis.offsetText.get_text()
#	ax.set_ylabel(r"$y_{utm}$ " + _yofflabel + " (m)", fontsize=7)
#	ax.set_xlabel(r"$x_{utm}$ " + _xofflabel + " (m)", fontsize=7)	

live_plot = animation.FuncAnimation(fig, update_plot, interval=30)
plt.axis('equal')
plt.show()
