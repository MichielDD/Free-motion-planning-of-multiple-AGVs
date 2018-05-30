"""
This file is part of the master thesis of Michiel De Deken, MSc. in Engineering: Logistics & Traffic @ KU Leuven: 

" Free motion planning of multiple AGVs using a spline based-approach
--- A multi-frame technique applied to a warehouse environment --- "

June 2018


Author: Michiel De Deken

"""
import numpy as np
from obstacle import *
from corridor import *
from basics.geometry import *
from basics.shape import * 
from globalplanner import *

class Environment(object):
	def __init__(self,width,height,obstacles=None):
		obstacles = obstacles or []
		self.width = width
		self.height = height

		# add obstacles
		self.obstacles = []
		self.corridors = []
		self.n_obs = 0 #nb of obs
		self.n_corridors = 0 #nb of corridors
		self.n_dim = 2 #nb of dimensions
		for obstacle in obstacles:
			self.add_obstacle(obstacle)
		
	def add_obstacle(self,obstacle):
		if isinstance(obstacle, list):
			for obst in obstacle:
				self.add_obstacle(obst)
		else:
			if obstacle.get_shape().n_dim != self.n_dim:
				raise ValueError('Not possible to combine ' +
								 str(obstacle.get_shape().n_dim) + 'D obstacle with ' +
								 str(self.n_dim) + 'D environment.')
			self.obstacles.append(obstacle)
			self.n_obs += 1

	def add_corridor(self,corridor):
		if isinstance(corridor, list):
			for corr in corridor:
				self.add_corridor(corr)
		else:
			if corridor.get_shape().n_dim != self.n_dim:
				raise ValueError('Not possible to combine ' +
								 str(obstacle.get_shape().n_dim) + 'D obstacle with ' +
								 str(self.n_dim) + 'D environment.')
			self.corridors.append(corridor)
			self.n_corridors += 1

	def get_obs_pos(self):
		return [o.get_pos() for o in self.obstacles]

	def get_obstacle(self,i):
		return self.obstacles[i]

	def get_canvas_limits(self):
		return [[0,width],[0,height]]

	def get_corridors(self):
		return self.corridors

	def get_corridor(self,i):
		return self.corridors[i]



class Warehouse(Environment):
	def __init__(self,width,height,rows,columns,aisle_x,aisle_y,rack_x,rack_y,buffer_x,buffer_y,dz_x,dz_y,n_cellsx,n_cellsy,obstacles=None): 	#dz = deliver zone
		super(Warehouse,self).__init__(width,height,obstacles=None)
		self.n_racks = 0
		self.n_rows = rows
		self.n_columns = columns
		self.dz_x = dz_x
		self.dz_y = dz_y
		self.aisle_x = aisle_x
		self.aisle_y = aisle_y
		self.corridor_centres = []
		for r in range(rows):
			for c in range(columns):
				rack = Obstacle([buffer_x+rack_x/2+c*aisle_x+c*rack_x,dz_y+buffer_y+rack_y/2+r*rack_y+r*aisle_y],[0.0,0.0],Rectangle(rack_x,rack_y)) #inti pos is center of rack
				self.add_rack(rack)

		self.initialize_corridors(width,height,rows,columns,aisle_x,aisle_y,rack_x,rack_y,buffer_x,buffer_y,dz_x,dz_y)
		self.blockedgrid = BlockedGrid(self,[n_cellsx,n_cellsy]).get_blocked_grid()

	def add_rack(self,rack):
		if isinstance(rack,list):
			for r in rack:
				self.add_rack(r)
		elif isinstance(rack,Obstacle):
			self.add_obstacle(rack)
			self.n_racks += 1 

	def get_n_racks(self):
		return self.n_racks

	def get_corridors_around_points(self,points):
		frames_of_waypoints = []
		for p in points:
			frames = []
			for corr in self.get_corridors():
				centre = corr.get_pos()
				w = corr.get_shape().width
				h = corr.get_shape().height
				if ((centre[0]-w/2 <= p[0] <= centre[0]+w/2) and (centre[1]-h/2 <= p[1] <= centre[1]+h/2)): #Important assumption: racks are not rotated ! 
					frames.append(corr)
			frames_of_waypoints.append(frames)
		return frames_of_waypoints



	def initialize_corridors(self,width,height,rows,columns,aisle_x,aisle_y,rack_x,rack_y,buffer_x,buffer_y,dz_x,dz_y):
		# Define corridors 
		h_centres = []
		h_corridor_size = []
		v_centres =[]
		v_corridor_size = []	
		for r in range(rows+1):
			for c in range(columns):
				xh = min((buffer_x+aisle_x+rack_x)/2.*(1+c),buffer_x+aisle_x+rack_x) +  max(0,(float(c)-0.5)*rack_x) +  max(0,(float(c)-1)*aisle_x) 
				yh = dz_y + min(buffer_y/2. * (r+1),buffer_y) + (r*rack_y )+ max(0,(float(r)-0.5)*aisle_y) 

				if (c == 0 or c == columns-1):
					h_corridor_width = buffer_x + aisle_x + rack_x
				else:
					h_corridor_width = aisle_x + rack_x*2.
				if (r == 0 or r == rows):
					h_corridor_height = buffer_y
				else:
					h_corridor_height = aisle_y

				h_centres.append([xh,yh])
				h_corridor_size.append([h_corridor_width,h_corridor_height])
	
		for r in range(rows):
			for c in range(columns+1):
				xv = min(buffer_x/2 *(1+c),buffer_x) + c*rack_x + max(0,(float(c)-0.5)*aisle_x) 
				yv = dz_y + min((buffer_y + rack_y+aisle_y)/2.*(1+r), buffer_y + rack_y + aisle_y) + max(0,(float(r)-0.5)*rack_y) +  max(0,(float(r)-1)*aisle_y) 
				
				if (c == 0 or c == columns):
					v_corridor_width= buffer_x
				else:
					v_corridor_width = aisle_x
				if (r == 0 or r == rows-1):
					v_corridor_height = rack_y+buffer_y+aisle_y
				else:
					v_corridor_height = 2.*aisle_y+rack_y

				v_centres.append([xv,yv])
				v_corridor_size.append([v_corridor_width,v_corridor_height])
		corridor_centres = h_centres + v_centres
		corridor_sizes = h_corridor_size + v_corridor_size
		offset = 0.0	


		for i in range(len(corridor_centres)):
			if i < len(h_centres):
				typ = 'horizontal'
			else:
				typ = 'vertical'		
			pos = corridor_centres[i]
			# limits = [xmin,ymin,xmax,ymax]
			limits = [corridor_centres[i][0]-corridor_sizes[i][0]/2 + offset,corridor_centres[i][1]- corridor_sizes[i][1]/2+offset,corridor_centres[i][0]+corridor_sizes[i][0]/2-offset,corridor_centres[i][1]+corridor_sizes[i][1]/2-offset]
			shape = Rectangle(width=corridor_sizes[i][0] -2*offset, height=corridor_sizes[i][1]-2*offset)
			print(pos,shape)

			self.add_corridor(Corridor(pos,shape,limits,typ,i))

		self.corridor_centres = corridor_centres # Purpose of making generic examples; to initialize start and end positions of vehicles

	def get_corridor(self,i):
		return self.corridors[i]

	def corridor_overlaps(self,corrid1,corrid2):
		corr1 = self.get_corridor(corrid1)
		corr2 = self.get_corridor(corrid2)

		return corr1.overlaps(corr2)


