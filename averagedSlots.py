
import pprint 
import itertools
import collections


class AveragedSlots(object):

	FIDELITY = 5   # range of degrees to average 

	def __init__(self):
		self.ave = 0.0
		self.count = 0.0
		self.flag = True
		self.x = 0    # raw value 
		self.debug =''  # for debuging 
		self.sum = 0
		self.slots = dict() # initialize an dict to store the buffers 
	
	def getLimits(self):
		self.l = self.x-(self.x%AveragedSlots.FIDELITY)              # get lower limit
		self.u = (self.x+AveragedSlots.FIDELITY) - (self.x+AveragedSlots.FIDELITY)%AveragedSlots.FIDELITY    # get upper limit 

	def update(self, x): 
		self.x = x
		if self.flag: 
			self.getLimits()
			self.ave = x
			self.sum = x
			self.count += 1.0
			self.flag = False 
			# the keys in the dict are the slots (by the lower limits) and contain arrays for their values
			if self.l in self.slots:
				self.slots[self.l].append(self.ave)
			else:
				self.slots[self.l] = [self.ave] 
			return self.ave
		else: 
			if (self.l <= x <= self.u):
				self.count += 1.0
				self.debug =  'count='+str(self.count) + '\tave=(' + str(self.sum) + '+'+str(x)+')/' + str(self.count)
				self.sum += x
				self.ave = (self.sum)/self.count 
				self.slots[self.l].append(self.ave) # slot already exist so no need to test if slot in dict 
				return self.ave
			else: 
				self.ave = x 
				self.sum = x
				self.getLimits()
				self.count = 1.0
				self.debug =  'count='+str(self.count) + '\tave=(' + str(self.sum) + ')/' + str(self.count)
				if self.l in self.slots:
					self.slots[self.l].append(self.ave)
				else:
					self.slots[self.l] = [self.ave] 
				return self.ave 

	"""
	This is a very complicated peice of code to print out buffers. Don't bother understanding it
	source: http://stackoverflow.com/a/9001529
	"""
	def __str__(self):
		s = ''
		sorted_slots = sorted(self.slots, key=lambda key: self.slots[key])
		s += ','.join(str(x) for x in sorted_slots) + '\n'
		prep_dict = collections.OrderedDict(sorted(self.slots.items()))
		prep_sorted_slots = [(str(x) for x in v) for k,v in prep_dict.iteritems()]
		for i in itertools.izip_longest(*prep_sorted_slots, fillvalue=" "):
			s += ','.join(i) + '\n'
			# print " ".join(i)
		return s




if __name__ == '__main__':
	x = range(1,50)
	ob = AveragedSlots()
	AveragedSlots.FIDELITY=5
	for i in x: 
		print i, '\t', ob.update(i), '\t', ob.l, '\t', ob.u
		# print ob.debug
	print '\n########################################################\n'
	pprint.pprint(ob.slots)
	print '\n########################################################\n'
	print ob
