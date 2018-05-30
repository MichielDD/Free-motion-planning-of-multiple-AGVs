"""
This file is part of the master thesis of Michiel De Deken, MSc. in Engineering: Logistics & Traffic @ KU Leuven: 

" Free motion planning of multiple AGVs using a spline based-approach
--- A multi-frame technique applied to a warehouse environment --- "

June 2018


Author: Michiel De Deken

"""
import casadi as cas
import numpy as np
from spline import *
from spline_extra import *


class GenericOCP(object):
	# This class builds generic problems of nb_of_veh vehicles. 
	# The parameters (such as starting state of the vehicles) are filled in, in a later phase of the algorithm. 

	def __init__(self,nb_of_veh,timehorizon):
		self.nb_of_veh = nb_of_veh
		self.T = timehorizon

		degree = 3
		self.knots = np.r_[np.zeros(degree), np.linspace(0, 1, 11), np.ones(degree)]
		self.basis = BSplineBasis(self.knots, degree)

		degree_hp = 3
		self.knots_hp = np.r_[np.zeros(degree_hp), np.linspace(0, 1, 11), np.ones(degree_hp)]
		self.basis_hp = BSplineBasis(self.knots_hp, degree_hp)

	
		self.initiate(self.nb_of_veh)

	def __repr__(self):
		return "Generic OCP with %s vehicles)" % (self.nb_of_veh)
	
	# def __eq__(self, other):
	# 	if isinstance(other, Vehicle):
	# 		return ((self.init_pos == other.init_pos) and (self.id == other.id))
	# 	else:
	# 		return False
	
	# def __ne__(self, other):
	# 	return (not self.__eq__(other))
	
	# def __hash__(self):
	# 	# Allows set operations on vehicles
	# 	return hash(self.__repr__())

	def get_id(self):
		return self.id

	def initiate(self,nb_of_veh):
		self.initiate_lists()
		self.create_sym_splines(nb_of_veh)
		self.create_hyperplanes(nb_of_veh)
		self.reset_constraints()
		self.define_constraints(nb_of_veh)
		self.create_problem(nb_of_veh)

	def initiate_lists(self):
		self.x_cfs_list = []
		self.y_cfs_list = []
		self.x_list = []
		self.y_list = []
		self.dx_list = []
		self.dy_list = []
		self.ddx_list = []
		self.ddy_list = []
		self.sx_cfs_list = []
		self.sy_cfs_list = []
		self.sx_list = []
		self.sy_list= [] 

		# self.ax_list= []
		# self.ax_cfs_list = []
		# self.ay_list= []
		# self.ay_cfs_list = []
		# self.b_list = []
		# self.b_cfs_list = []
		self.lmbd_cfs_list = []
		self.lmbd_list = []

		self.corr_x_min_list = []
		self.corr_y_min_list = []
		self.corr_x_max_list = []
		self.corr_y_max_list = []

		self.corr_start_list = []
		self.corr_end_list = []
		self.corr_hp_list = []
		self.treshold_list = []

		self.corr_v0_list = []

		self.maxVel_list = []
		self.maxAcc_list = []
		self.radius_list = []
		self.overlap_list = []

	def reset_constraints(self):
		self.con, self.lb, self.ub = [], [], []


	def create_sym_splines(self,nb_of_veh):

		for i in range(nb_of_veh):
			## creating symbolic splines
			# x,y position trajectory and derivatives
			x_cfs = cas.SX.sym('x_cfs'+str(i), len(self.basis))
			y_cfs = cas.SX.sym('y_cfs'+str(i), len(self.basis))
			x = BSpline(self.basis, x_cfs)
			y = BSpline(self.basis, y_cfs)
			dx = x.derivative()
			dy = y.derivative()
			ddx = x.derivative(2)
			ddy = y.derivative(2)
			# slack variable spline for norm1 objective reformulation
			sx_cfs = cas.SX.sym('sx_cfs'+str(i), len(self.basis))
			sy_cfs = cas.SX.sym('sy_cfs'+str(i), len(self.basis))
			sx = BSpline(self.basis, sx_cfs)
			sy = BSpline(self.basis, sy_cfs)

			self.x_cfs_list.append(x_cfs)
			self.y_cfs_list.append(y_cfs)
			self.x_list.append(x)
			self.y_list.append(y)
			self.dx_list.append(dx)
			self.dy_list.append(dy)
			self.ddx_list.append(ddx)
			self.ddy_list.append(ddy)
			self.sx_cfs_list.append(sx_cfs)
			self.sy_cfs_list.append(sy_cfs)
			self.sx_list.append(sx)
			self.sy_list.append(sy)

			corr_x_min = cas.SX.sym('corr_x_min'+str(i),1)
			corr_y_min = cas.SX.sym('corr_y_min'+str(i),1)
			corr_x_max = cas.SX.sym('corr_x_max'+str(i),1)
			corr_y_max = cas.SX.sym('corr_y_max'+str(i),1)
			corr_start = cas.SX.sym('corr_start'+str(i),2)
			corr_end = cas.SX.sym('corr_end'+str(i),2)
			corr_hp = cas.SX.sym('corr_hp'+str(i),4)
			corr_treshold = cas.SX.sym('corr_treshold'+str(i),2)
		

			self.corr_x_min_list.append(corr_x_min)
			self.corr_y_min_list.append(corr_y_min)
			self.corr_x_max_list.append(corr_x_max)
			self.corr_y_max_list.append(corr_y_max)
		
			self.corr_start_list.append(corr_start)
			self.corr_end_list.append(corr_end)
			self.corr_hp_list.append(corr_hp)
			self.treshold_list.append(corr_treshold)
		
			#t = cas.SX.sym('T'+str(i),1)
			#self.T.append(t)

			corr_v0 = cas.SX.sym('corr_v0'+str(i),2)
			self.corr_v0_list.append(corr_v0)

			maxVel = cas.SX.sym('maxVel'+str(i),1)
			self.maxVel_list.append(maxVel)

			maxAcc = cas.SX.sym('maxAcc'+str(i),1)
			self.maxAcc_list.append(maxAcc)

			radius = cas.SX.sym('radius'+str(i),1)
			self.radius_list.append(radius)

			overlapbool = cas.SX.sym('overlapbool'+str(i),nb_of_veh-i)
			self.overlap_list.append(overlapbool)


	def create_hyperplanes(self,nb_of_veh):
		for i in range(nb_of_veh):
			l_cfs = cas.SX.sym('l_cfs'+str(i), len(self.basis_hp))
			l = BSpline(self.basis_hp, l_cfs)
			self.lmbd_cfs_list.append(l_cfs)
			self.lmbd_list.append(l)

			# Outcommented lines are for the creation of separating hyperplanes 
			# with moving obstacles. This is not fully worked out in this version 
			# of the algorithm. 
			
			# ax_cfs = cas.SX.sym('ax_cfs'+str(i), len(self.basis_hp))
			# ay_cfs = cas.SX.sym('ay_cfs'+str(i), len(self.basis_hp))
			# b_cfs = cas.SX.sym('b_cfs'+str(i), len(self.basis_hp))
			# ax = BSpline(self.basis_hp, ax_cfs)
			# ay = BSpline(self.basis_hp, ay_cfs)
			# b = BSpline(self.basis_hp, b_cfs)
			# self.ax_cfs_list.append(ax_cfs)
			# self.ay_cfs_list.append(ay_cfs)
			# self.b_cfs_list.append(b_cfs)
			# self.ax_list.append(ax)
			# self.ay_list.append(ay)
			# self.b_list.append(b)

	def define_constraints(self,nb_of_veh):
		for i in range(nb_of_veh):
			# velocity & acceleration inequality constraints
			self.con += [self.dx_list[i] - self.maxVel_list[i]*self.T, -self.dx_list[i] - self.maxVel_list[i]*self.T, self.ddx_list[i] - self.maxAcc_list[i]*self.T**2, -self.ddx_list[i] - self.maxAcc_list[i]*self.T**2]
			self.con += [self.dy_list[i] - self.maxVel_list[i]*self.T, -self.dy_list[i] - self.maxVel_list[i]*self.T, self.ddy_list[i] - self.maxAcc_list[i]*self.T**2, -self.ddy_list[i] - self.maxAcc_list[i]*self.T**2]
			self.lb += [-cas.inf for _ in range(8)]
			self.ub += [0. for _ in range(8)]

			# slack variable constraints
			self.con += [self.x_list[i] - self.corr_end_list[i][0] - self.sx_list[i], -(self.x_list[i] - self.corr_end_list[i][0]) - self.sx_list[i]]
			self.con += [self.y_list[i] - self.corr_end_list[i][1] - self.sy_list[i], -(self.y_list[i] - self.corr_end_list[i][1]) - self.sy_list[i]]
			self.lb += [-cas.inf for _ in range(4)]
			self.ub += [0. for _ in range(4)]

			self.con += [-self.x_list[i] + self.corr_x_min_list[i], self.x_list[i] - self.corr_x_max_list[i] , -self.y_list[i]+self.corr_y_min_list[i], self.y_list[i] - self.corr_y_max_list[i]]
			self.lb += [-cas.inf for _ in range(4)] 
			self.ub += [0. for _ in range(4)]

		#Collision avoidance constraints with moving obstacles. 
		# basis_obs = BSplineBasis(np.r_[0., 0., 1., 1.], 1)
		# obs_trajx_list = []
		# obs_trajy_list = []
		# v_obs= [0,0] #TODO 
		# for h in range(self.n_racks):
		# 	obs_trajx = BSpline(basis_obs, [self.pos_obst[h][0], self.pos_obst[h][0]+self.T*v_obs[0]])
		# 	obs_trajy = BSpline(basis_obs, [self.pos_obst[h][1], self.pos_obst[h][1]+self.T*v_obs[1]])
		# 	print(obs_trajx)
		# 	obs_trajx_list.append(obs_trajx)
		# 	obs_trajy_list.append(obs_trajy)


		#Inter vehicles avoidance constraints
		if nb_of_veh > 1:
			for i in range(nb_of_veh):
				for j in range(i+1,nb_of_veh):
					self.con += [self.overlap_list[i][j-i]*(-((self.x_list[i]-self.x_list[j])**2+(self.y_list[i]-self.y_list[j])**2)+(self.radius_list[i]+self.radius_list[j])**2)] 
					self.lb += [-cas.inf]
					self.ub += [0.]


		# Pivoting separating hyperplane constraints
		for i in range(nb_of_veh):
			self.con += [self.radius_list[i] + (1-self.lmbd_list[i])*self.corr_hp_list[i][2]*(self.x_list[i]-self.corr_hp_list[i][0]) + self.lmbd_list[i]*self.corr_hp_list[i][3]*(self.y_list[i]-self.corr_hp_list[i][1])]
			self.lb += [-cas.inf]
			self.ub += [0.]

			self.con += [self.lmbd_list[i]]
			self.lb += [0.]
			self.ub += [1.]

		# spline constraint relaxation: express spline constraints on their coefficients
		self.lb = [l*np.ones(c.coeffs.shape[0]) for l, c in zip(self.lb, self.con)]
		self.ub = [u*np.ones(c.coeffs.shape[0]) for u, c in zip(self.ub, self.con)]
		self.con = [c.coeffs for c in self.con]


		for i in range(nb_of_veh):
			# begin and terminal equality constraints
			# terminal constraints are ignored (comment out). This is achieved by definition of the objective. 
			# Imposing the terminal constraints would facility the problem to become infeasible. 
			self.con += [self.x_list[i](0) - self.corr_start_list[i][0], self.y_list[i](0) - self.corr_start_list[i][1]]
			self.lb += [0 for _ in range(2)]
			self.ub += [0. for _ in range(2)]
			# self.con += [self.x_list[i](1) - self.corr_end_list[i][0], self.y_list[i](1) - self.corr_end_list[i][1]]
			# self.lb += [-cas.inf for _ in range(2)]
			# self.ub += [0. for _ in range(2)]
			
			# constraints to have smooth starting of speed and acceleration each frame combination
			self.con += [self.dx_list[i](0)-self.corr_v0_list[i][0]*self.T, self.dy_list[i](0)-self.corr_v0_list[i][1]*self.T]
			self.lb += [0. for _ in range(2)]
			self.ub += [0. for _ in range(2)]


	def create_problem(self,nb_of_veh):
		# Objective is 1 norm of the slacks 
		obj = sum(definite_integral(self.sx_list[i], 0, 1) + definite_integral(self.sy_list[i], 0, 1) for i in range(nb_of_veh))
		var = cas.vertcat()
		par = cas.vertcat()
		# Setting up the optimization variables (symbolic) for each vehicle. 
		for i in range(nb_of_veh):
			var = cas.vertcat(var, self.x_cfs_list[i], self.y_cfs_list[i], self.sx_cfs_list[i], self.sy_cfs_list[i])
		for i in range(nb_of_veh):
			var = cas.vertcat(var,self.lmbd_cfs_list[i])
		# Setting up the parameter set (symbolic) for each vehicle. These values will be filled in later on, in each solver iteration.
		for i in range(nb_of_veh):
 			par = cas.vertcat(par,self.corr_start_list[i],self.corr_end_list[i],self.corr_x_min_list[i],self.corr_y_min_list[i],self.corr_x_max_list[i],self.corr_y_max_list[i],self.corr_hp_list[i],self.treshold_list[i])
			par = cas.vertcat(par,self.corr_v0_list[i],self.maxVel_list[i],self.maxAcc_list[i],self.radius_list[i],self.overlap_list[i])

		con = cas.vertcat(*self.con)
		nlp = {'x': var, 'f': obj, 'p':par, 'g': con}
		solver = cas.nlpsol('solver', 'ipopt', nlp, {'ipopt.tol': 1e-5, 'ipopt.linear_solver': 'ma57'})

		self.solver = solver
		self.nlp = nlp 

	# def pickle_GenericProblem(p):
	# 	return GenericProblem, p.nb_of_veh, p.T





