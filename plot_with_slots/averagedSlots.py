




class AveragedSlots(object):

	DEGREE_FREEDOM = 5   # range of degrees to average 

	def __init__(self):
		self.ave = 0.0
		self.count = 0.0
		self.flag = True
		self.x = 0    # raw value 
		self.debug =''  # for debuging 
		self.sum = 0
	
	def getLimits(self):
		self.l = self.x-(self.x%AveragedSlots.DEGREE_FREEDOM)              # get lower limit
		self.u = (self.x+AveragedSlots.DEGREE_FREEDOM) - (self.x+AveragedSlots.DEGREE_FREEDOM)%AveragedSlots.DEGREE_FREEDOM    # get upper limit 

	def update(self, x): 
		self.x = x
		if self.flag: 
			self.getLimits()
			self.ave = x
			self.sum = x
			self.count += 1.0
			self.flag = False 
			return self.ave
		else: 
			if (self.l <= x <= self.u):
				self.count += 1.0
				self.debug =  'count='+str(self.count) + '\tave=(' + str(self.sum) + '+'+str(x)+')/' + str(self.count)
				self.sum += x
				self.ave = (self.sum)/self.count 
				return self.ave
			else: 
				self.ave = x 
				self.sum = x
				self.getLimits()
				self.count = 1.0
				self.debug =  'count='+str(self.count) + '\tave=(' + str(self.sum) + ')/' + str(self.count)
				return self.ave 


if __name__ == '__main__':
	x = range(1,50)
	ob = AveragedSlots()
	for i in x: 
		print i, '\t', ob.update(i)
		# print i, '\t', ob.l, '\t', ob.u
		# print ob.debug