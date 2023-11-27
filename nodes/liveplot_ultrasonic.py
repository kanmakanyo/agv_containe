#!/usr/bin/env python

########## SIMPLE ULTRASONIC LIVE PLOT
###### Written by DimasAP (github.com/dispectra)
###### init. code 2021-02-22

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
from geometry_msgs.msg import TwistStamped


########### INITIAL DATA
rospy.init_node('liveplot_ultrasomic')
print("[INFO] rosnode 'liveplot_ultrasomic' initialized")

# waypoints_path = rospy.get_param('~waypoints_path', 'wp_monev_baru.npy')
# waypoints_path = '2021-02-16-12-20-13.npy'
# waypoints_path = os.path.abspath(sys.path[0] + '/../src/waypoints/waypoints/' + waypoints_path)
# print("waypoints file:", waypoints_path)
# wp = np.load(waypoints_path)
# wp = wp[:10]
# wp[:,0] = wp[:,0]-60
# # wp[:,1] = wp[:,1]-60

class UltrasonicSensor():
	def __init__(self, x, y, theta):
		self.x = x
		self.y = y
		self.theta = theta
		self.reading = 0
		self.fov = np.deg2rad(75)

	def updateReading(self, reading):
		self.reading = reading

lim_reading = 5
car_length = 3.85
car_width = 0.90
car_front_to_sensor1 = 1.30
car_front_to_sensor2 = 0.05
car_right_to_sensor2 = 0.05

car_drawing = np.array([
	[car_width/2, car_length/2],
	[-car_width/2, car_length/2],
	[-car_width/2, -car_length/2],
	[car_width/2, -car_length/2],
	[car_width/2, car_length/2]
])

us_sensor = []
# us sensor 1
us_sensor.append(UltrasonicSensor(car_width/2, car_length/2-car_front_to_sensor1, 0))
# us sensor 2
us_sensor.append(UltrasonicSensor(car_width/2-car_right_to_sensor2, car_length/2-car_front_to_sensor2, np.pi/4))
# us sensor 3
us_sensor.append(UltrasonicSensor(0, car_length/2, np.pi/2))
# us sensor 4
us_sensor.append(UltrasonicSensor(-1*(car_width/2-car_right_to_sensor2), car_length/2-car_front_to_sensor2, np.pi*3/4))
# us sensor 5
us_sensor.append(UltrasonicSensor(-1*(car_width/2), car_length/2-car_front_to_sensor1, np.pi))
# us sensor 6
us_sensor.append(UltrasonicSensor(-1*(car_width/2), -1*(car_length/2-car_front_to_sensor1), np.pi))
# us sensor 7
us_sensor.append(UltrasonicSensor(-1*(car_width/2-car_right_to_sensor2), -1*(car_length/2-car_front_to_sensor2), np.pi*5/4))
# us sensor 8
us_sensor.append(UltrasonicSensor(0, -1*(car_length/2), np.pi*3/2))
# us sensor 9
us_sensor.append(UltrasonicSensor((car_width/2-car_right_to_sensor2), -1*(car_length/2-car_front_to_sensor2), np.pi*7/4))
# us sensor 10
us_sensor.append(UltrasonicSensor(car_width/2, -1*(car_length/2-car_front_to_sensor1), 0))

#dummy data
for i in range(10):
	us_sensor[i].updateReading(1)

########### UPDATE DATA
def cbNanoUltrasonicFront(msg):
	global us_sensor

	us_sensor[0].updateReading(msg.twist.linear.x/100 if msg.twist.linear.x/100 < lim_reading else lim_reading)
	us_sensor[1].updateReading(msg.twist.linear.y/100 if msg.twist.linear.y/100 < lim_reading else lim_reading)
	us_sensor[2].updateReading(msg.twist.linear.z/100 if msg.twist.linear.z/100 < lim_reading else lim_reading)
	us_sensor[3].updateReading(msg.twist.angular.x/100 if msg.twist.angular.x/100 < lim_reading else lim_reading)
	us_sensor[4].updateReading(msg.twist.angular.y/100 if msg.twist.angular.y/100 < lim_reading else lim_reading)

def cbNanoUltrasonicRear(msg):
	global us_sensor

	us_sensor[5].updateReading(msg.twist.linear.x/100 if msg.twist.linear.x/100 < lim_reading else lim_reading)
	us_sensor[6].updateReading(msg.twist.linear.y/100 if msg.twist.linear.y/100 < lim_reading else lim_reading)
	us_sensor[7].updateReading(msg.twist.linear.z/100 if msg.twist.linear.z/100 < lim_reading else lim_reading)
	us_sensor[8].updateReading(msg.twist.angular.x/100 if msg.twist.angular.x/100 < lim_reading else lim_reading)
	us_sensor[9].updateReading(msg.twist.angular.y/100 if msg.twist.angular.y/100 < lim_reading else lim_reading)

rospy.Subscriber('/nano_ultrasonic_front', TwistStamped, cbNanoUltrasonicFront)
rospy.Subscriber('/nano_ultrasonic_rear', TwistStamped, cbNanoUltrasonicRear)
print("[INFO] Subscribed to topics")

########### PLOT DATA
print("[INFO] Starting to plot..")
mpl.rcParams['toolbar'] = 'None'
fig = plt.figure(figsize=(3.5, 3.5))
fig.canvas.set_window_title('Ultrasonic Sensor Live Plot')
ax = plt.gca()
ax.format_coord = lambda x, y: ''

def circle_points(center_x, center_y, rad, angle_start=0, angle_end=2*np.pi, 
				  inc_mode='point_num', inc_param=10):
	points = []
	if (inc_mode == 'point_num'):
		for i in range(inc_param):
			theta = (angle_end - angle_start)/(inc_param-1)*i
			x = center_x + (rad * np.cos(theta+angle_start))
			y = center_y + (rad * np.sin(theta+angle_start))
			points.append([x,y])

	return np.array(points)

def update_plot(i):
	# REF https://pythonprogramming.net/live-graphs-matplotlib-tutorial/
	global us_sensor

	# plt.clf()
	ax.clear()

	ax.plot(car_drawing[:,0], car_drawing[:,1], zorder=1, c='k')
	ax.text(0, car_length/4, 'FRONT', ha='center')
	for i in range(10):
		ax.scatter(us_sensor[i].x, us_sensor[i].y, c='g', zorder=2)
		ax.annotate(' us'+str(i+1), (us_sensor[i].x, us_sensor[i].y))
		circle_line = circle_points(us_sensor[i].x, us_sensor[i].y, us_sensor[i].reading, 
				 angle_start=us_sensor[i].theta-us_sensor[i].fov/2,
				 angle_end=us_sensor[i].theta+us_sensor[i].fov/2)
		circle_fill = np.insert(circle_line, len(circle_line), [us_sensor[i].x, us_sensor[i].y], axis=0)
		if (us_sensor[i].reading == lim_reading):
			ax.plot(circle_line[:,0], circle_line[:,1], label='us'+str(i+1), ls='--', c='b', alpha=0.5)
			ax.fill(circle_fill[:,0], circle_fill[:,1], c='b', alpha=0.05, lw=0)
		else:
			ax.plot(circle_line[:,0], circle_line[:,1], label='us'+str(i+1), ls='-', c='r', alpha=0.5)
			ax.fill(circle_fill[:,0], circle_fill[:,1], c='r', alpha=0.05, lw=0)


	# ax.legend(loc="upper right", fontsize=7)

	ax.set_xlim(-10, 10)
	ax.set_ylim(-10, 10)
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