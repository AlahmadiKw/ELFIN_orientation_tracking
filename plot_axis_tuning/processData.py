import sys, serial
from math import *
import numpy as np
import matplotlib.pyplot as plt
import argparse
from averagedSlots import *


class ProcessData:
	alpha_val = .4
	acc_sensitivity = 1100
	mag_calibrate = [-9.1302, 66.5231, -62.0005]
	def __init__(self, fidelity):
		self.xAccFilteredOld = 0
		self.yAccFilteredOld = 0
		self.zAccFilteredOld = 0
		self.xMagFilteredOld = 0
		self.yMagFilteredOld = 0
		self.zMagFilteredOld = 0
		AveragedSlots.FIDELITY = fidelity
		self.pitch_ave = AveragedSlots()
		self.roll_ave  = AveragedSlots()
		self.yaw_ave   = AveragedSlots()

	################### NO MORE USED ###################
	def filterData(self):  
		# filter Acc data 
		xAccFiltered = self.xAccFilteredOld + ProcessData.alpha_val * (self.xAcc - self.xAccFilteredOld)
		yAccFiltered = self.yAccFilteredOld + ProcessData.alpha_val * (self.yAcc - self.yAccFilteredOld)
		zAccFiltered = self.zAccFilteredOld + ProcessData.alpha_val * (self.zAcc - self.zAccFilteredOld)

		self.xAccFilteredOld = xAccFiltered
		self.yAccFilteredOld = yAccFiltered
		self.zAccFilteredOld = zAccFiltered

		self.xAcc = xAccFiltered
		self.yAcc = yAccFiltered
		self.zAcc = zAccFiltered

		# filter mag data 
		xMagFiltered = self.xMagFilteredOld + ProcessData.alpha_val * (self.xMag - self.xMagFilteredOld)
		yMagFiltered = self.yMagFilteredOld + ProcessData.alpha_val * (self.yMag - self.yMagFilteredOld)
		zMagFiltered = self.zMagFilteredOld + ProcessData.alpha_val * (self.zMag - self.zMagFilteredOld)

		self.xMagFilteredOld = xMagFiltered
		self.yMagFilteredOld = yMagFiltered
		self.zMagFilteredOld = zMagFiltered

		self.xMag = xMagFiltered
		self.yMag = yMagFiltered
		self.zMag = zMagFiltered

	def normalizeData(self):


		self.xAcc /= ProcessData.acc_sensitivity
		self.yAcc /= ProcessData.acc_sensitivity
		self.zAcc /= ProcessData.acc_sensitivity

		self.xMag -= ProcessData.mag_calibrate[0]
		self.yMag -= ProcessData.mag_calibrate[1]
		self.zMag -= ProcessData.mag_calibrate[2]

		norm = sqrt(pow(self.xMag,2) + pow(self.yMag,2) + pow(self.zMag,2))
		self.xMag /= norm
		self.yMag /= norm
		self.zMag /= norm

	def computeAngles(self):
		self.pitch = atan2(self.xAcc, sqrt(pow(self.yAcc,2) + pow(self.zAcc,2)))
		self.roll  = atan2(self.yAcc, sqrt(pow(self.xAcc,2) + pow(self.zAcc,2)))
		self.yaw   = atan2(self.yMag*cos(self.roll) + self.zMag*sin(self.roll),
			         self.xMag*cos(self.pitch) + self.yMag*sin(self.pitch)*sin(self.roll) - self.zMag*cos(self.roll)*sin(self.pitch))
		self.pitch *= 180/pi
		self.roll  *= 180/pi
		self.yaw   *= 180/pi

	def process(self, rawData):
		self.xAcc = rawData[0]
		self.yAcc = rawData[1]
		self.zAcc = rawData[2]
		self.xMag = rawData[3]
		self.yMag = rawData[4]
		self.zMag = rawData[5]
		# self.filterData()
		self.normalizeData()
		self.computeAngles()

		self.pitch = self.pitch_ave.update(self.pitch)
		self.roll = self.roll_ave.update(self.roll)
		self.yaw = self.yaw_ave.update(self.yaw)

		return (self.pitch, self.roll, self.yaw)

if __name__ == '__main__':
	# in_file = sys.argv[1]
	# out_file = sys.argv[2]

	# sensor = ProcessData()
	# line_out = []
	# with open(in_file) as in_f:
	# 	with open(out_file, 'w') as out_f:
	# 		for line in in_f:
	# 			values = [float(val) for val in line.split(',')]
	# 			(pitch, roll, yaw) = sensor.process(values[3:len(values)])
	# 			final = [str(val) for val in [pitch, roll, yaw]]
	# 			line_out.extend(final)
	# 			line_out = ' '.join(line_out)
	# 			line_out += '\n'
	# 			out_f.write(line_out)
	# 			line_out = []


	# soa =np.array( [0,0,5,10]) 
	# # X,Y,U,V = zip(*soa)
	# (X, Y, U, V) = soa.tolist() 
	# print X,Y,U,V 
	# plt.figure()
	# plt.grid(b=True)
	# ax = plt.gca()
	# ax.quiver(X,Y,U,V,angles='xy',scale_units='xy',scale=1)
	# ax.set_xlim([-20,20])
	# ax.set_ylim([-20,20])
	# plt.draw()
	# plt.show()
	parser = argparse.ArgumentParser()
	parser.add_argument("portname", help="name of portname (com for windows or /dev/tty.usbmodem for mac)",
	                    type=str)
	parser.add_argument("--output", help="save data to file",
                    type=str)
	args = parser.parse_args()
	print args.portname
	if args.output=='mak':
		print "mak mak mak"






	
