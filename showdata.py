################################################################################
# showdata.py
#
# plots magnetometer data 
#
# 
# real time plotting is credited to electronut.in:
# https://gist.github.com/electronut/5641933
#
################################################################################

import sys, serial
import io
import numpy as np
from time import sleep
from collections import deque
from matplotlib import pyplot as plt
from processData import ProcessData
from math import cos, sin, pi, copysign

import matplotlib.cbook as cbook

from  mpl_toolkits.axisartist.grid_helper_curvelinear import GridHelperCurveLinear
from mpl_toolkits.axisartist import Subplot

from mpl_toolkits.axisartist import SubplotHost, \
     ParasiteAxesAuxTrans

import  mpl_toolkits.axisartist.angle_helper as angle_helper
from matplotlib.projections import PolarAxes
from matplotlib.transforms import Affine2D

# class that holds analog data for N samples
class AngLesbuf:
	# constr
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

# plot class
class AnglesPlot:
	# constr
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
	''' plots heading angle and signal strength in polar coor '''
	def __init__(self):
		plt.ion()
		self.fig = plt.figure(num=2, figsize=(10,7))
		# plt.grid(b=True)
		# self.ax = plt.gca()
		# self.quiverLine = self.ax.quiver(0,0,100,100,angles='xy',scale_units='xy',scale=1)
		# self.ax.set_xlim([-150,150])
		# self.ax.set_ylim([-150,150])
		# self.annotation = self.ax.annotate('init', xy=(100+5, 100+5))
		    # PolarAxes.PolarTransform takes radian. However, we want our coordinate
		# system in degree
		tr = Affine2D().scale(np.pi/180., 1.) + PolarAxes.PolarTransform()

		# polar projection, which involves cycle, and also has limits in
		# its coordinates, needs a special method to find the extremes
		# (min, max of the coordinate within the view).

		# 20, 20 : number of sampling points along x, y direction
		extreme_finder = angle_helper.ExtremeFinderCycle(50, 50,
		                                                 lon_cycle = 360,
		                                                 lat_cycle = None,
		                                                 lon_minmax = None,
		                                                 lat_minmax = (0, np.inf),
		                                                 )

		grid_locator1 = angle_helper.LocatorDMS(24)
		# Find a grid values appropriate for the coordinate (degree,
		# minute, second).

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
		# let bottom axis shows ticklabels for 2nd coordinate (radius)
		self.ax1.axis["bottom"].get_helper().nth_coord_ticks=0
		self.ax1.axis["left"].get_helper().nth_coord_ticks=0
		temp =  self.ax1.set_title('Signal strength & heading polar plots')
		temp.set_y(1.05) 


		self.ax1.grid(True)

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

		ticks = np.linspace(0, 255, 6)
		offset = np.zeros([1,255])
		for i in range(1,5):
			self.ax1.annotate(str(ticks[i]),size=10, xy=(ticks[i], -15))
			blah = self.ax1.plot(ticks[i],0, 'bo')

			self.ax1.annotate(str(ticks[i]),size=10, xy=(5, ticks[i]))
			blah = self.ax1.plot(0,ticks[i], 'bo')

		bbox_props = dict(boxstyle="round", fc="w", ec="0.5", alpha=0.9)
		# self.annotation = self.ax1.annotate('init',size=20, xy=(100, 100), bbox = bbox_props)
		self.annotation = plt.figtext(0.02, 0.9, 'rssi = ', size=20, alpha = 0.9, bbox = bbox_props)
		self.Freq = plt.figtext(0.85, 0.85, 'freq = ???', size=10, alpha = 0.9, bbox = bbox_props)
		self.Freq = plt.figtext(0.85, 0.9, 'Horizontal Plane', size=10, alpha = 0.9, bbox = bbox_props)

		self.quiverLine = self.ax1.quiver(0,0,50,50,angles='xy',scale_units='xy',scale=1)
		

		self.ax1.set_aspect(1.)
		self.ax1.set_xlim(-255, 255)
		self.ax1.set_ylim(-255, 255)
		

		self.xdata = []
		self.ydata = []
		self.polarline, = self.ax1.plot(self.xdata,self.ydata)
		# self.polarline.set_data([],[])

		



	def update(self, signalStrength, yaw):
		U = signalStrength*cos(yaw*pi/180)
		V = signalStrength*sin(yaw*pi/180)
		self.xdata.append(U)
		self.ydata.append(V)
		# self.ax1.plot(self.xdata,self.ydata)
		self.polarline.set_data(self.xdata,self.ydata)
		self.quiverLine.set_UVC(U,V)
		self.annotation.set_text('rssi = ' + str(signalStrength))
		plt.draw()


# main() function
def main():
	if(len(sys.argv) != 3):
		print 'Example usage: python showdata.py "/dev/tty.usbmodem411"'
		exit(1)
	#strPort = '/dev/tty.usbserial-A7006Yqh'
	strPort = sys.argv[1];

	# initialize raw data processing 
	sensor = ProcessData()


	print 'plotting data...'

	# open serial port
	# ser = serial.Serial(port = strPort, baudrate = 9600, timeout= 2) 
	# line = ser.readline()   # this is NEEDED before writing tx 
	# nbytes = ser.write("tx".encode('ascii'))



	# whichPlot = 'polar'/'cartesian' 
	whichPlot = sys.argv[2]; 
	if (whichPlot == 'cartesian'):
		# plot parameters
		angles = AngLesbuf(100)
		anglesPlot = AnglesPlot(angles)
		flag = True
	elif (whichPlot == 'polar'):
		polar = PolarPlot()
		flag = False
	else:
		print 'wrong arguments (polar/cartesian)'
		exit(1)

	

	# while True:
	# 	try:
	# 		if ser.readable(): 
	# 			line = ser.readline()
	# 			# print line 
	# 			data = [float(val) for val in line.split(',')]
	# 			# print data
	# 			if(len(data)==9):
	# 				(pitch, roll, yaw) = sensor.process(data[3:len(data)])
	# 				signalStrength = data[0]
	# 				if flag:
	# 					angles.add([pitch, roll, yaw])
	# 					anglesPlot.update(angles)
	# 				else:
	# 					polar.update(signalStrength, yaw)
	# 		# sleep(.06)
	# 	except ValueError:
	# 		if not line: 
	# 			userInput = raw_input('receive data again? or else exit (y/n)\n')
	# 			if userInput.lower() == 'y':
	# 				polar.xdata = []
	# 				polar.ydata = []
	# 				nbytes = ser.write("tx".encode('ascii'))
	# 			else:
	# 				print 'exiting loop'
	# 				break
	# 		else: 
	# 			print 'bad data', line
	# 	except KeyboardInterrupt:
	# 		print 'exiting'
	# 		break





	# DEBUGING CODE
	sensor = ProcessData()
	in_file = sys.argv[1]
	with open(in_file) as in_f:
		for line in in_f: 
			try:
				data = [float(val) for val in line.split(',')]
				if(len(data)==9):
					(pitch, roll, yaw) = sensor.process(data[3:len(data)])
					signalStrength = data[0]
					if flag:
						angles.add([pitch, roll, yaw])
						anglesPlot.update(angles)
					else:
						polar.update(signalStrength, yaw)
			except ValueError:
				print 'bogus data'
			except (KeyboardInterrupt,SystemExit):
				print 'exiting'
				# plt.close('all')
				break



	plt.show(block=True)



	# close serial
	ser.flush()
	ser.close()

# call main
if __name__ == '__main__':
	main()