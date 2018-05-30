import numpy as np
from spline import *
from spline_extra import *


class Obstacle(object):
	def __init__(self,initpos,velocity,shape):
		self.initpos = initpos
		self.shape = shape
		self.velocity = velocity

	def get_pos(self):
		return self.initpos

	def get_velocity(self):
		return self.velocity

	def get_shape(self):
		return self.shape
	


# class Rack(Obstacle):
# 	def __init__(self,initpos,width,height,shape, velocity = [0.0,0.0]):
# 		super(Rack,self).__init__(initpos,velocity,shape)
# 		self.width = width
# 		self.height = height
# 		self.vertices = np.array([[0., 0.], [0., self.height], [self.width, self.height], [self.width, 0]])
# 		self.vertices_centr = np.array([[self.width/2,self.width/2,-self.width/2,-self.width/2],[self.height/2,-self.height/2,-self.height/2,self.height/2]])

# 	def get_center(self):
# 		return [self.initpos[0] + self.width/2 , self.initpos[1] + self.height/2]

# 	def get_vertices(self):
# 		return self.vertices

# 	def get_vertices_centr(self):
# 		return self.vertices_centr

# class MovingPolygonObst(Obstacle):
# 	def __init__(self,initpos,vertices,velocity, shape):
# 		super(MovingObst,self).__init__(initpos,velocity,shape)
# 		self.vertices = vertices

# 	def get_vertices(self):
# 		return self.vertices

	# def get_trajectories(self,T):
	# 	# returns linear trajectories in form of a b spline
	# 	basis_obs = BSplineBasis(np.r_[0., 0., 1., 1.], 1)
	# 	obs_trajx = BSpline(basis_obs, [self.getpos[0], self.getpos[0]+T*self.get_velocity[0]])
	# 	obs_trajy = BSpline(basis_obs, [self.getpos[1], self.getpos[1]+T*self.get_velocity[1]])






