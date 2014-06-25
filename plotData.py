################################################################################
# plotData.py
#
# plots magnetometer data 
#
# 
# real time waveform plotting is credited to electronut.in:
# https://gist.github.com/electronut/5641933
#
# other classes are based on examples from matplotlib documentation
################################################################################

import sys, serial
import io
import numpy as np
from time import sleep
from collections import deque
from matplotlib import pyplot as plt
from processData import ProcessData
from math import cos, sin, pi, copysign
import argparse

import matplotlib.cbook as cbook

from  mpl_toolkits.axisartist.grid_helper_curvelinear import GridHelperCurveLinear
from mpl_toolkits.axisartist import Subplot
from mpl_toolkits.mplot3d import proj3d

from mpl_toolkits.axisartist import SubplotHost, \
     ParasiteAxesAuxTrans

import  mpl_toolkits.axisartist.angle_helper as angle_helper
from matplotlib.projections import PolarAxes
from matplotlib.transforms import Affine2D

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D


class AngLesbuf:
	""" saves new data to buffer for plotting angels waveform """
	def __init__(self, maxLen):
		self.ax = deque([0.0]*maxLen)
		self.ay = deque([0.0]*maxLen)
		self.az = deque([0.0]*maxLen)
		self.maxLen = maxLen

	# ring buffer
	def addToBuf(self, buf, val):
		if len(buf) < self.maxLen:
			buf.append(val)
		else:
			buf.pop()
			buf.appendleft(val)

	# add data
	def add(self, data):
		assert(len(data) == 3)
		self.addToBuf(self.ax, data[0])
		self.addToBuf(self.ay, data[1])
		self.addToBuf(self.az, data[2])


class AnglesPlot:
	""" plots anges in waveform """
	def __init__(self, angles):
		# set plot to animated
		plt.ion() 
		plt.figure(num=1, figsize=(10,7))
		plt.subplots_adjust(wspace=0.5)
		plt.subplot(311)
		self.axline, = plt.plot(angles.ax)
		plt.ylim([-100, 100])
		plt.grid()
		plt.title('pitch')
		plt.xlabel('samples')
		plt.ylabel('degrees')
		plt.subplot(312)
		self.ayline, = plt.plot(angles.ay)
		plt.ylim([-100, 100])
		plt.grid()
		plt.title('roll')
		plt.xlabel('samples')
		plt.ylabel('degrees')
		plt.subplot(313)
		self.azline, = plt.plot(angles.az)
		plt.ylim([-190, 190])
		plt.grid()
		plt.title('yaw')
		plt.xlabel('samples')
		plt.ylabel('degrees')

	# update plot
	def update(self, angles):
		self.axline.set_ydata(angles.ax)
		self.ayline.set_ydata(angles.ay)
		self.azline.set_ydata(angles.az)
		plt.draw()

class PolarPlot:
	''' plots heading angle and signal strength in polar coor in 2d'''
	def __init__(self):
		plt.ion()
		self.fig = plt.figure(num=2, figsize=(10,7))
		tr = Affine2D().scale(np.pi/180., 1.) + PolarAxes.PolarTransform()
		# 20, 20 : number of sampling points along x, y direction
		extreme_finder = angle_helper.ExtremeFinderCycle(50, 50,
		                                                 lon_cycle = 360,
		                                                 lat_cycle = None,
		                                                 lon_minmax = None,
		                                                 lat_minmax = (0, np.inf),
		                                                 )
		grid_locator1 = angle_helper.LocatorDMS(24)
		tick_formatter1 = angle_helper.FormatterDMS()
		# And also uses an appropriate formatter.  Note that,the
		# acceptable Locator and Formatter class is a bit different than
		# that of mpl's, and you cannot directly use mpl's Locator and
		# Formatter here (but may be possible in the future).
		grid_helper = GridHelperCurveLinear(tr,
		                                    extreme_finder=extreme_finder,
		                                    grid_locator1=grid_locator1,
		                                    tick_formatter1=tick_formatter1
		                                    )

		self.ax1 = SubplotHost(self.fig, 1, 1, 1, grid_helper=grid_helper)
		
		# make ticklabels of right and top axis visible.
		self.ax1.axis["right"].major_ticklabels.set_visible(True)
		self.ax1.axis["top"].major_ticklabels.set_visible(True)
		self.ax1.axis["left"].major_ticklabels.set_visible(True)

		# let right axis shows ticklabels for 1st coordinate (angle)
		self.ax1.axis["right"].get_helper().nth_coord_ticks=0
		# let bottom axis shows ticklabels for 1st coordinate (angle)
		self.ax1.axis["bottom"].get_helper().nth_coord_ticks=0
		self.ax1.axis["left"].get_helper().nth_coord_ticks=0
		temp =  self.ax1.set_title('Signal strength & heading polar plots')
		temp.set_y(1.05) 

		self.ax1.grid(True)

		# insert x and y axises
		self.ax = self.fig.add_subplot(self.ax1)
		self.ax1.spines['left'].set_position('center')
		self.ax1.spines['right'].set_color('red')
		self.ax1.spines['bottom'].set_position('center')
		self.ax1.spines['top'].set_color('none')
		self.ax1.spines['left'].set_smart_bounds(True)
		self.ax1.spines['bottom'].set_smart_bounds(True)
		self.ax1.xaxis.set_ticks_position('bottom')
		self.ax1.yaxis.set_ticks_position('left')
		self.ax1.axhline(linewidth=2, color='blue')
		self.ax1.axvline(linewidth=2, color='blue')

		# label x and y axises manually 
		ticks = np.linspace(0, 255, 6)
		offset = np.zeros([1,255])
		for i in range(1,5):
			self.ax1.annotate(str(ticks[i]),size=10, xy=(ticks[i], -15))
			blah = self.ax1.plot(ticks[i],0, 'bo')

			self.ax1.annotate(str(ticks[i]),size=10, xy=(5, ticks[i]))
			blah = self.ax1.plot(0,ticks[i], 'bo')

		# annotate figure 
		bbox_props = dict(boxstyle="round", fc="w", ec="0.5", alpha=0.9)
		# self.annotation = self.ax1.annotate('init',size=20, xy=(100, 100), bbox = bbox_props)
		self.annotation = plt.figtext(0.02, 0.9, 'rssi = ', size=20, alpha = 0.9, bbox = bbox_props)
		self.Freq = plt.figtext(0.85, 0.85, 'freq = ???', size=10, alpha = 0.9, bbox = bbox_props)
		self.Freq = plt.figtext(0.85, 0.9, 'Horizontal Plane', size=10, alpha = 0.9, bbox = bbox_props)

		# initialize arrow 
		self.quiverLine = self.ax1.quiver(0,0,50,50,angles='xy',scale_units='xy',scale=1)		
		self.ax1.set_aspect(1.)
		self.ax1.set_xlim(-255, 255)
		self.ax1.set_ylim(-255, 255)
		
		# initialize mesh plot
		self.xdata = []
		self.ydata = []
		self.polarline, = self.ax1.plot(self.xdata,self.ydata)

	def update(self, signalStrength, yaw):
		U = signalStrength*cos(yaw*pi/180)
		V = signalStrength*sin(yaw*pi/180)
		self.xdata.append(U)
		self.ydata.append(V)
		self.polarline.set_data(self.xdata,self.ydata)
		self.quiverLine.set_UVC(U,V)
		self.annotation.set_text('rssi = ' + str(signalStrength))
		plt.draw()



class Polar3D():
	''' plots orientation and signal strength in polar coor in 3d'''
	def __init__(self):
		plt.ion()
		self.fig = plt.figure(num=3, figsize=(12,7))
		self.ax = self.fig.gca(projection='3d')
		self.ax.set_xlim(-255, 255)
		self.ax.set_ylim(-255, 255)
		self.ax.set_zlim(-255, 255)
		axisline = np.linspace(-255, 255, 6)
		zeros =  np.zeros((1,6)).tolist()[0]
		self.ax.plot(axisline, zeros, zeros, 'b', linewidth=2)
		self.ax.plot(zeros, axisline, zeros, 'b', linewidth=2)
		self.ax.plot(zeros, zeros, axisline, 'b', linewidth=2)

		bbox_props = dict(boxstyle="round", fc="w", ec="0.5", alpha=0.9)
		self.annotation = plt.figtext(0.02, 0.9, 'rssi = ', size=15, alpha = 0.9, bbox = bbox_props)
		self.Freq = plt.figtext(0.91, 0.85, 'freq = ???', size=10, alpha = 0.9, bbox = bbox_props)
		self.Freq = plt.figtext(0.91, 0.9, 'Horizontal Plane', size=10, alpha = 0.9, bbox = bbox_props)

		# quiver lines (there is no quiver 3d so I draw the vector manually 
		# using plot of two points (0 to rho)
		self.referencedir = np.array([1, 0, 0])
		self.polarline, = self.ax.plot([0, self.referencedir[0]] , [0, self.referencedir[1]], [0, self.referencedir[2]], 'k', linewidth=2)
		# since its a T shaped, these are the two smaller lines 
		self.wing1 = np.array([0, 1, 0])
		self.wing2 = np.array([0, -1, 0])
		self.polarline, = self.ax.plot([0, self.wing1[0]] , [0, self.wing1[1]], [0, self.wing1[2]], 'k', linewidth=2)
		self.polarline, = self.ax.plot([0, self.wing2[0]] , [0, self.wing2[1]], [0, self.wing2[2]], 'k', linewidth=2)
		# initialize x,y,z buffers for mesh plotting as well
		self.xdata = []
		self.ydata = []
		self.zdata = []
		self.polarline2, = self.ax.plot(self.xdata,self.ydata, self.zdata, 'r')
		
	def update(self, rho, pitch, roll, yaw):
		pitch *= pi/180
		roll  *= pi/180
		yaw   *= pi/180 
		# rotation matrix (wikipedia)
		rotation_mat = np.array([[ cos(roll)*cos(yaw) ,  cos(pitch)*sin(yaw)+sin(pitch)*sin(roll)*cos(yaw), sin(pitch)*sin(yaw)-cos(pitch)*sin(roll)*cos(yaw)], 
			      				 [ -cos(roll)*sin(yaw),  cos(pitch)*cos(yaw)-sin(pitch)*sin(roll)*sin(yaw), sin(pitch)*cos(yaw)+cos(pitch)*sin(roll)*sin(yaw)],
			      				 [ sin(roll)          ,  -sin(pitch)*cos(roll)                            , cos(pitch)*cos(roll)]])
		xyz = np.dot(rotation_mat,self.referencedir)
		wing1 = np.dot(rotation_mat,self.wing1) * rho*.5
		wing2 = np.dot(rotation_mat,self.wing2) * rho*.5
		xyz = xyz * rho
		self.polarline.set_data([wing1[0],0, xyz[0], 0, wing2[0]] , [wing1[1],0, xyz[1], 0, wing2[1]])
		self.polarline.set_3d_properties([wing1[2],0, xyz[2], 0, wing2[2]])
		wing1 = np.dot(rotation_mat,self.wing1)
		wing2 = np.dot(rotation_mat,self.wing2)

		self.xdata.append(xyz[0])
		self.ydata.append(xyz[1])
		self.zdata.append(xyz[2])
		self.polarline2.set_data(self.xdata , self.ydata)
		self.polarline2.set_3d_properties(self.zdata)

		self.annotation.set_text('rssi = ' + str(rho))
		plt.draw()

# main() function
def main():
	# -----------------------------------------------------------
	# parse command line args
	# -----------------------------------------------------------
	parser = argparse.ArgumentParser()
	parser.add_argument("--portname", help="for realtime: name of portname (com* for windows or /dev/tty.usbmodem* for mac)",
	                    type=str)
	parser.add_argument("--datafile", help="for offline: name of sensor data file to plot3d from",
	                    type=str)

	parser.add_argument("--output", help="save data to OUTPUT",
                    type=str)
	args = parser.parse_args()

	out_file = ''
	if args.output:
		out_file = args.output

	#strPort = '/dev/tty.usbserial-A7006Yqh'
	strPort = args.portname;

	# initialize raw data processing 
	sensor = ProcessData()

	if (not args.portname) and (not args.datafile):
		print 'please specify either portname of data file (see python plotData.py -h for usage'
		exit(1)


	print 'plotting data...'

	# -----------------------------------------------------------
	# 2D plotting
	# -----------------------------------------------------------
	# open serial port
	if strPort:
		ser = serial.Serial(port = strPort, baudrate = 9600, timeout= 2) 
		line = ser.readline()   # this is NEEDED before writing tx 
		nbytes = ser.write("tx".encode('ascii'))

		polar = PolarPlot()
		lines = []
		while True:
			try:
				if ser.readable(): 
					line = ser.readline()
					# print line 
					data = [float(val) for val in line.split(',')]
					# print data
					if(len(data)==9):
						(pitch, roll, yaw) = sensor.process(data[3:len(data)])
						signalStrength = data[0]
						# if flag:
						# 	angles.add([pitch, roll, yaw])
						# 	anglesPlot.update(angles)
						# else:
						polar.update(signalStrength, yaw)
						if out_file:
							fileline = ' '.join([str(val) for val in data])
							lines.append(fileline)
			except ValueError:
				if not line: 
					userInput = raw_input('receive data again? or else exit (y/n)? ')
					if userInput.lower() == 'y':
						polar.xdata = []
						polar.ydata = []
						nbytes = ser.write("tx".encode('ascii'))
					else:
						print 'exiting loop'
						break
				else: 
					print 'bad data', line
			except KeyboardInterrupt:
				print 'exiting'
				break
	# save data to output is user specifies output args 
	if out_file:
		lines = '\n'.join(lines)
		with open(out_file, 'w') as f:
			f.writelines(lines)

	# -----------------------------------------------------------
	# 3D plotting
	# -----------------------------------------------------------
	if args.datafile:
		polar = Polar3D()
		sensor = ProcessData()
		in_file = args.datafile
		with open(in_file) as in_f:
			for line in in_f: 
				try:
					data = [float(val) for val in line.split(',')]
					if(len(data)==9):
						(pitch, roll, yaw) = sensor.process(data[3:len(data)])
						signalStrength = data[0]
						polar.update(signalStrength, pitch, roll, yaw)
				except ValueError:
					print 'bogus data', line 
				except (KeyboardInterrupt,SystemExit):
					print 'exiting'
					# plt.close('all')
					break

	plt.show(block=True)



	# close serial
	if strPort:
		ser.flush()
		ser.close()

# call main
if __name__ == '__main__':
	main()