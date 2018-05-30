"""
This file is part of the master thesis of Michiel De Deken, MSc. in Engineering: Logistics & Traffic @ KU Leuven: 

" Free motion planning of multiple AGVs using a spline based-approach
--- A multi-frame technique applied to a warehouse environment --- "

June 2018


Author: Michiel De Deken

"""
class Corridor(object):
	# Corridor class. 
	# A corridor is a rectangular shape in which no warehouse racks are present.
	def __init__(self,initpos,shape,limits,typ,ident):
		self.initpos = initpos
		self.shape = shape
		self.limits = limits
		self.type = typ
		self.id = ident

	def __repr__(self):
		return "Corridor(%s, %s)" % (self.initpos, self.id)
	
	def __eq__(self, other):
		if isinstance(other, Corridor):
			return ((self.initpos == other.initpos) and (self.id == other.id))
		else:
			return False
	
	def __ne__(self, other):
		return (not self.__eq__(other))
	
	def __hash__(self):
		# Allows set operations on corridors
		return hash(self.__repr__())

	def get_pos(self):
		#Returns centre of corridor
		return self.initpos

	def get_type(self):
		return self.type

	def get_shape(self):
		return self.shape

	def get_limits(self):
		# limits = [xmin,ymin,xmax,ymax]
		return self.limits

	def get_id(self):
		return self.id


	def overlaps(self,other):
		# Check if two rectangles overlap
		if self == other:
			return True

		else:
			xmin1,ymin1,xmax1,ymax1 = self.get_limits()
			xmin2,ymin2,xmax2,ymax2 = other.get_limits()
			overlapbool = (xmin1 < xmax2 and xmax1 > xmin2 and ymax1 > ymin2 and ymin1 < ymax2) 
			return overlapbool

