"""
This file is part of the master thesis of Michiel De Deken, MSc. in Engineering: Logistics & Traffic @ KU Leuven: 

" Free motion planning of multiple AGVs using a spline based-approach
--- A multi-frame technique applied to a warehouse environment --- "

June 2018


Author: Michiel De Deken

"""


class Vehicle(object):
	# Vehicle class 
	def __init__(self,ide,init_pos,target,max_vel,max_acc,rad):
		self.id = ide
		self.init_pos = init_pos
		self.target = target
		self.max_acc = max_acc
		self.max_vel = max_vel
		self.rad = rad
		self.parameters = []
		self.time = 0.
		self.curr_framenb = 0
		self.startx = init_pos[0]
		self.starty = init_pos[1]
		self.isRouted = False
		self.v0x = 0.
		self.v0y = 0.
		self.route_sampledx = []
		self.route_sampledy = []
		self.samplesx = []
		self.samplesy = []
		self.samplesdx = []
		self.samplesdy = []
		self.horizonPointer = 0
		self.totalPointer = 0
		self.midPointer = 0
		self.globalMidPointer = 0

	def __repr__(self):
		return "Vehicle id=%s, pos=%s)" % (self.id, self.init_pos)
	
	def __eq__(self, other):
		if isinstance(other, Vehicle):
			return ((self.init_pos == other.init_pos) and (self.id == other.id))
		else:
			return False
	
	def __ne__(self, other):
		return (not self.__eq__(other))
	
	def __hash__(self):
		# Allows set operations on vehicles
		return hash(self.__repr__())

	def get_id(self):
		return self.id

	def get_rad(self):
		return self.rad


	def get_start_pos(self):
		# returns [x_init_pos, y_init_pos]
		return [self.startx,self.starty]

	def get_target(self):
		# returns [x_target, y_target]
		return self.target

	def get_vmax(self):
		return self.max_vel

	def get_amax(self):
		return self.max_acc

	def add_parameters(self,param):
		self.parameters.append(param)

	def get_param_of_frame(self,framenb):
		return self.parameters[framenb]

	def get_nb_of_frames(self):
		return len(self.parameters)

	def get_curr_time(self):
		return self.time 

	def set_new_time(self,newtime):
		self.time = newtime

	def get_curr_corridors(self):
		return self.parameters[self.get_curr_frame()].corrids

	def get_curr_frame(self):
		return self.curr_framenb

	def increase_curr_framenb(self):
		self.curr_framenb += 1

	def set_new_start(self,startx,starty,v0x,v0y):
		self.startx = startx
		self.starty = starty
		self.v0x, self.v0y = v0x, v0y

	def extend_sampled_route(self,samplesx,samplesy):
		self.route_sampledx.extend(samplesx)
		self.route_sampledy.extend(samplesy)

	# Depreciated 
	# def overlaps_corrids(self,other,environment):
	# 	if len(self.get_curr_corridors()) == 1:
	# 		id1_self = self.get_curr_corridors()[0]
	# 	else:
	# 		id1_self,id2_self = self.get_curr_corridors()
	# 		corr2self = environment.get_corridor(id2_self)
	# 	if len(other.get_curr_corridors()) == 1:
	# 		id1_other = self.get_curr_corridors()[0]
	# 	else:
	# 		id1_other,id2_other = other.get_curr_corridors()
	# 		corr2other = environment.get_corridor(id2_other)
	# 	corr1self = environment.get_corridor(id1_self)
	# 	corr1other = environment.get_corridor(id1_other)
	# 	if corr1other == corr1self or corr1other.overlaps(corr1self):
	# 		return True
	# 	if len(other.get_curr_corridors()) > 1:
	# 		if (corr1other == corr2self) and (corr2other == corr1self): #or corr2other.overlaps(corr1self):
	# 			return True
	# 	else: 
	# 		return False

	def overlaps_corridors(self,other,environment):
		ids_self = self.get_curr_corridors()
		ids_other = other.get_curr_corridors()
		for corr in ids_self:
			corr1 = environment.get_corridor(corr)
			for corrother in ids_other:
				corr2 = environment.get_corridor(corrother)
				if corr1.overlaps(corr2):
					return True
		return False


	def update(self,startx,starty,newtime,v0x,v0y,samplesx,samplesy,p):
		# Update this vehicle with new parameters, as a result of the switch to another multiframe 
		# of a (!) vehicle, thus not necessarily THIS vehicle. 
		# The new parameters will serve as an input for futher optimizations (receding horizon strategy)

		self.set_new_start(startx,starty,v0x,v0y)
		self.set_new_time(newtime)


		self.globalMidPointer = self.convert_local_to_globalpointer(self.midPointer)
		if self.convert_local_to_globalpointer(p) >= self.globalMidPointer:
			# print("here")
			# print(self.get_id())
			# print(p)
			# print(self.convert_local_to_globalpointer(p))
			# print(self.globalMidPointer)
			if self.get_curr_frame() < self.get_nb_of_frames()-1:
				self.increase_curr_framenb()
		self.increaseHorizon(p)
		self.is_at_destination(samplesx,samplesy)

	def is_at_destination(self,samplesx,samplesy):
		diff = (self.startx - self.target[0])**2 +  (self.starty - self.target[1])**2 
		if diff < 0.05: # Treshold to avoid further optimizations if target is almost reached
			self.isRouted = True
			self.append_route(samplesx,samplesy)
		else:
			self.extend_sampled_route(samplesx,samplesy)

	def setPointers(self,p):
		self.midPointer = p

	def increaseHorizon(self,p):
		self.horizonPointer += p

	def append_route(self,samplesx,samplesy):
		# append the sampled route of this vehicle till the end with the given samples
		currently = len(self.route_sampledx)
		print(currently,"currently")
		self.route_sampledx.extend(samplesx[:1000-currently]) #1000 equals n_animframes
		self.route_sampledy.extend(samplesy[:1000-currently])

	def convert_otherpointer_to_local(self,other,otherpointer):
		# otherpointer is a local midpointer of a frameset of another vehicle 'other'
		if self == other:
			return self.midPointer
		globalpointer = other.horizonPointer + otherpointer
		localpointer = globalpointer - self.horizonPointer
		return abs(localpointer)

	def convert_local_to_globalpointer(self,localpointer):
		return self.horizonPointer + localpointer

	def get_isRouted(self):
		return self.isRouted