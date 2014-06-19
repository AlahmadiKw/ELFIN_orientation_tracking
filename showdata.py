################################################################################
# showdata.py
#
# Display analog data from Arduino using Python (matplotlib)
# 
# electronut.in
#
################################################################################

import sys, serial
import io
import numpy as np
from time import sleep
from collections import deque
from matplotlib import pyplot as plt
from processData import ProcessData
from math import cos, sin, pi

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
		plt.figure(num=2, figsize=(10,7))
		plt.grid(b=True)
		self.ax = plt.gca()
		self.quiverLine = self.ax.quiver(0,0,100,100,angles='xy',scale_units='xy',scale=1)
		self.ax.set_xlim([-150,150])
		self.ax.set_ylim([-150,150])


	def update(self, signalStrength, yaw):
		U = signalStrength*cos(yaw*pi/180)
		V = signalStrength*sin(yaw*pi/180)
		self.quiverLine.set_UVC(U,V)
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
	# 				nbytes = ser.write("tx".encode('ascii'))
	# 			else:
	# 				print 'exiting loop'
	# 				break
	# 		else: 
	# 			print 'bad data', line
	# 	except KeyboardInterrupt:
	# 		print 'exiting'
	# 		break



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