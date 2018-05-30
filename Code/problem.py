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
import matplotlib.pyplot as plt
from matplotlib import animation
import matplotlib.patches as patches
from wh_environment import *
from obstacle import *
from vehicle import *
from globalplanner import *
from parameters import *
from GenericOCP import *

import time 

class Problem(object):
	# Initiate a problem 
	def __init__(self,environment,vehicles,n_cellsx,n_cellsy,moviename,show=True,writemovie=False,timehorizon=25.0):
		self.environment = environment
		self.vehicles = vehicles
		self.n_veh = len(self.vehicles)
		self.n_cellsx = n_cellsx
		self.n_cellsy = n_cellsy
		self.show = show
		self.moviename = moviename
		self.writemovie = writemovie
		self.T = timehorizon

		if isinstance(environment,Warehouse):
			self.n_racks = environment.get_n_racks()
			self.obst_vert = environment.get_obstacle(0).get_shape().get_vertices() 
			self.pos_obst = environment.get_obs_pos()

		self.route_sampled_x = [[] for i in range(self.n_veh)]
		self.route_sampled_y = [[] for i in range(self.n_veh)]
		self.waypointslist = [[] for i in range(self.n_veh)]

		self.static_vehicles_x = []
		self.static_vehicles_y = []
		self.n_animframes = 1000
		self.infeasible_cnt = 0
		self.genericOCPs = [-999 for i in range(self.n_veh)]

		# Start solving the complete OCP
		self.solve_main()


	# Code underneath was a try to group vehicles into clusters, 
	# in which the agents have to take into account the trajectories of the others 
	# during optimization. Because of the complexity of defining these separated groups, and 
	# due to the induced difficulty considering guaranteeing the vehicles still 
	# live in the same time space, this approach is abondoned. The code 
	# is not deleted because in a further stage of the project this may function as 
	# a start-up idea.  

	# def make_groups(self,vehicles):
	# 	groups = []
	# 	last_veh = min(vehicles, key=lambda veh: veh.globalMidPointer+veh.midPointer)
	# 	last_veh_group = [last_veh]
	# 	for veh in vehicles:
	# 		if veh != last_veh and last_veh.overlaps_corrids(veh,self.environment):
	# 			last_veh_group.append(veh) 
	# 	groups.append(last_veh_group)
	# 	flatgroup = [item for group in groups for item in group]

	# 	while len(flatgroup) != len(vehicles):
	# 		outsiders = []
	# 		for veh in vehicles: 
	# 			if veh not in flatgroup:
	# 				outsiders.append(veh)
	# 		last_veh = min(outsiders, key=lambda veh: veh.globalMidPointer+veh.midPointer)
	# 		new_group = [last_veh]
	# 		for veh in outsiders:
	# 			if veh != last_veh and last_veh.overlaps_corrids(veh,self.environment):
	# 				new_group.append(veh) 
	# 		groups.append(new_group)
	# 		flatgroup = [item for group in groups for item in group]
	# 	return groups 


	def solve_main(self):
		# This function is the main thread across the algorithm.

		timeinfo = [[] for v in range(self.n_veh)]

		# First, identify for each vehicle its waypoints, such that the parameters can be determined. 
		# Secondly, solve the trajectory, considering 1 vehicle at a time. 
		# The latter is important to have a first guess about the midPointers (i.e. the pointers at which the vehicle
		# switches from multiframe.)

		for i in range(self.n_veh):
			veh_id_list = [i]
			self.provide_waypoints(veh_id_list)
			for index,i in enumerate(veh_id_list):
				self.set_corridor_parameters(i) #xstart,ystart,xend,yend,xmin,ymin,xmax,ymax,xhp,yhp,treshx,treshy, etc. 
			#self.solve_problem(veh_id_list) #individual

		t0 = time.time()
		while len(self.get_active_vehicles()) > 0: #As long as there are active vehicles, trajectories have to be optimized
			flatgroup=self.get_active_vehicles()
			groupflatid = [veh.get_id() for veh in flatgroup]
			self.solve_problem(groupflatid) #Solve the complete set of active vehicles all at once. 
			# An important remark has to be made, considering the above:
			# Although the solvers considers all the active vehicles at the same time, 
			# the vehicles do not share all the constraints, or in other words stated,
			# the constraints are made vehicle specific, by using multiframes 

			# The vehicle with the earliest multiframe switch is chosen
			# A pure midPointer comparison is not adequate, because it is a local value. The global 
			# pointer (i.e. time unit equivalent) has to be considered as well (globalMidPointer)
			minPointerVeh = min(flatgroup, key=lambda veh: veh.globalMidPointer+veh.midPointer)

			# Get the pointer of the vehicle with the earliest multiframe switch. 
			p = minPointerVeh.midPointer

			minTime = minPointerVeh.time
			print("p is",p)

			# Sample all the active vehicles up to pointer p and update, such that the new vehicles' states 
			# can be used in the subsequent optimization step.
			for veh in flatgroup:
				route_sampled_x = veh.samplesx[:p]
				route_sampled_y = veh.samplesy[:p]
				startx,starty,v0x,v0y = veh.samplesx[p],veh.samplesy[p],veh.samplesdx[p],veh.samplesdy[p]
				veh.update(startx,starty,minTime,v0x,v0y,route_sampled_x,route_sampled_y,p)

		for item in range(len(self.vehicles)):
			self.route_sampled_x[item],self.route_sampled_y[item] = self.vehicles[item].route_sampledx,self.vehicles[item].route_sampledy 

		# An infeasible iteration does not necessarily mean that the trajectory is fault. 
		# It might be that the infeasibility occurs in the second frame of a multiframe, and by means 
		# of a receding horizon strategy this might be solved in a subsequent step.
		# however, stay conscientious when analyzing the output (plot)
		t1 = time.time()

		total = t1-t0
		print("Time to complete (excluding global planner) = ",total," seconds.")

		print("Number of infeasible iterations= ",self.infeasible_cnt)
		# Time to show results
		self.plot_sol(range(self.n_veh))

		# Final check if there are any collisions present
		self.detect_conflicts(self.route_sampled_x,self.route_sampled_y)

	def all_vehicles_routed(self):
		for veh in self.vehicles:
			if not veh.get_isRouted():
				return False
		return True

	def get_active_vehicles(self):
		active_veh = []
		for veh in self.vehicles:
			if not veh.get_isRouted():
				active_veh.append(veh)
		return active_veh

	def get_inactive_vehicles(self):
		inactive_veh = []
		for veh in self.vehicles:
			if veh.get_isRouted():
				inactive_veh.append(veh)
		return inactive_veh


	def detect_conflicts(self,x,y):
		# Detect if there is any overlap between the vehicles 
		for v1 in range(len(x)):
			for v2 in range(v1+1,len(x)):
				min_len = min(len(x[v1]),len(x[v2]))
				for i in range(min_len):
					if (x[v1][i]-x[v2][i])**2+(y[v1][i]-y[v2][i])**2 <= (self.vehicles[v1].get_rad()+self.vehicles[v2].get_rad())**2:
						print(x[v1][i],y[v1][i],v1,v2)
		return 

	def solve_problem(self,veh_id_list):
		# Solve the local trajectories of vehicles with their vehicle.id in veh_id_list

		# The algorithm makes uses of GenericProblems (See class). A GenericProblem is defined by the number
		# of vehicles it has to optimize the trajectories of. Instead of generating these problems for each
		# sub-problem over and over again, the code only sets up such a problem if needed, and stores it into 
		# a genericProblems list. 
		problemIndex = len(veh_id_list) - 1

		if self.genericOCPs[problemIndex] == -999: # The generic problem is not yet built
			self.genericOCPs[problemIndex] = GenericOCP(len(veh_id_list),self.T) # build the problem

		problem = self.genericOCPs[problemIndex] 
		solver = problem.solver
		nlp = problem.nlp
		var0 = np.zeros(nlp['x'].shape[0])

		# build a matrix (N by N), in which N is the number of considered vehicles in this problem
		# Overlap_matrix[i,j] is 1 if there is an overlap of frames between vehicle i and j.
		# this matrix is used later on such that not all vehicles have to have collision avoidance 
		# constraints with other vehicles. 
		overlap_matrix = [[0 for i in range(len(veh_id_list))] for j in range(len(veh_id_list))]
		for index,i in enumerate(veh_id_list):
			veh1 = self.vehicles[i]
			for indexj,j in enumerate(veh_id_list):
				if indexj != index:
					veh2 = self.vehicles[j]
					if veh1.overlaps_corridors(veh2,self.environment):
						overlap_matrix[index][indexj] = 1

		timeinfo = [[] for v in range(len(veh_id_list))]

	
		var0init = var0
		var0 = var0init

		par0 = []
		ps = [] #parameterset
		f = []
		# Setting the parameters wich will be passed to the GenericProblem, used for this local OCP 
		for index,i in enumerate(veh_id_list):
			f.append(self.vehicles[i].get_curr_frame())
			param = self.vehicles[i].get_param_of_frame(f[index])
			param.set_start_pos(self.vehicles[i].startx, self.vehicles[i].starty)
			param.set_initial_velocity(self.vehicles[i].v0x,self.vehicles[i].v0y)
			param.set_maxVelAccRad(self.vehicles[i].max_vel,self.vehicles[i].max_acc,self.vehicles[i].get_rad())
			#Give only the upper diagonal part of the matrix through to reduce the parameters in the problem. 
			#This is possible because overlap_matrix is symmetrical. 
			param.set_overlaplist(overlap_matrix[index][(index):]) 
			par0 += param.convert_to_problem_par()
			# print('vehicle params of ', i)
			# print(param.startx, param.starty)
			# print(param.endx, param.endy)
			# print(param.xmin,param.ymin,param.xmax,param.ymax)
			# print(param.xhp,param.yhp)
			ps.append(param)



		# 2 lines underneath set up initial guess for the coefficients as straight line from origin to destination		
		for index,i in enumerate(veh_id_list):
			var0[(index*4)*len(problem.basis):((index*4)+1)*len(problem.basis)] = np.linspace(ps[index].startx, ps[index].endx, len(problem.basis)) # initial guess
			var0[((index*4)+1)*len(problem.basis):((index*4)+2)*len(problem.basis)] = np.linspace(ps[index].starty, ps[index].endy, len(problem.basis))
		lbg=cas.vertcat(*problem.lb)

		sol = solver(x0=var0, p= par0, lbg=cas.vertcat(*problem.lb), ubg=cas.vertcat(*problem.ub))
		stat = solver.stats()

		# Counting infeasible solver iteration. If solution is infeasible, still succeed, because infeasibility may lie in 
		# second frame of multi-frame combination. Might be solved later on. 
		if stat['return_status'] != 'Solve_Succeeded':
			if stat['return_status'] != 'Solved_To_Acceptable_Level':
				self.infeasible_cnt +=1

		x_list, y_list,dx_list,dy_list,ddx_list,ddy_list = self.extract_solution(sol,veh_id_list)

		print("Solving vehicles: ",veh_id_list)
		for index,i in enumerate(veh_id_list):
			if self.vehicles[i].curr_framenb == self.vehicles[i].get_nb_of_frames()-1:
				# If the vehicle reached its last multiframe, the boolean is set to True
				# this bool is used when detecting the threshold values to which sampling has to be done
				# If the vehicle reached this last multiframe, the pointer where to sample corresponds to the 
				# end, e.g. 999 if 1000 frames (n_animframes) are used. 
				lastFramebool = True
			else: 
				lastFramebool = False
			self.vehicles[i].samplesx,self.vehicles[i].samplesy,self.vehicles[i].samplesdx,self.vehicles[i].samplesdy,fp,self.vehicles[i].time= self.find_starting_point_next_framecombo(index,ps[index].treshold_x,ps[index].treshold_y,x_list,y_list,dx_list,dy_list,lastFramebool)

			self.vehicles[i].setPointers(fp)
			print("Veh ",self.vehicles[i].get_id(),"has pointer ",fp,"and globalMidPointer",self.vehicles[i].globalMidPointer )

		return sol,timeinfo


	

	def find_starting_point_next_framecombo(self,i,tresholdx,tresholdy,x_list,y_list,dx_list,dy_list,lastFramebool):
		# Detects when a vehicle switches of multiframe F1-F2 to F2-F3. This occurs upon crossing the threshold values.
		# How the transition is made depends on the kind of turn, orientation of the individual frames
		# This switching point, from now on pointer called, is important to know till when the sampling has 
		# to executed. Because starting from the pointer a new trajectory is calculated. (cfr. Receding horizon strategy)
		samplenb = self.n_animframes
		infinity = samplenb - 1
		t = self.T
		time = np.linspace(0, t, samplenb)
		samplesx = x_list[i](time/t)
		samplesy = y_list[i](time/t)
		samplesdx = dx_list[i](time/t)
		samplesdy = dy_list[i](time/t)
		index = samplenb-1
		if lastFramebool == False:
			if tresholdx != infinity: # current corridor combo is h->v, so xlist
				if samplesx[-1] > samplesx[0]: #ASSUMPTION lists is strictly ascending or descending
					for j,item in enumerate(samplesx):
						if item > tresholdx:
							index = j
							break
				else:
					for j,item in enumerate(samplesx):
						if item < tresholdx:
							index = j
							break
			else:# tresholdy != infinity:: # current corridor combo is v->h, so ylist
				if samplesy[-1] > samplesy[0]: #ASSUMPTION lists is strictly ascending or descending
					for j,item in enumerate(samplesy):
						if item > tresholdy:
							index = j
							break
				else:
					for j,item in enumerate(samplesy):
						if item < tresholdy:
							index = j
							break
			# else tresholdx == infinity and tresholdy == infinity:
			# 	index = samplenb-1
		if index == 0:
			# If index is 0, this means we are already beyond treshold. This occurs when a vehicle is in its last corridor of its 
			# last framecombo. Therefore the index, later used as a pointer has to be maximum. In order to prevent the vehicle 
			# is chosen as minPointerveh, but still will be taken into account for further opitmizations.
			index = infinity

		return samplesx,samplesy,samplesdx,samplesdy,index,time[index]*t

	def extract_solution(self,sol,veh_id_list):
		# Extracts the solution out of sol. This boils down to retransforming the B-spline coefficients to the position, velocity and 
		# acceleration vectors.
		problemIndex = len(veh_id_list) - 1
		problem = self.genericOCPs[problemIndex] 
		var = sol['x']
		start = 4*self.n_veh*len(problem.basis)
		n_veh = len(veh_id_list)
		x_cfs_list = [np.array(var[(i*4)*len(problem.basis):((i*4)+1)*len(problem.basis)]).ravel() for i in range(n_veh)]
		y_cfs_list = [np.array(var[((i*4)+1)*len(problem.basis):((i*4)+2)*len(problem.basis)]).ravel() for i in range(n_veh)]
		x_list = [BSpline(problem.basis, x_cfs_list[i]) for i in range(n_veh)]
		y_list = [BSpline(problem.basis, y_cfs_list[i]) for i in range(n_veh)]
		dx_list = [(1./self.T)*x_list[i].derivative() for i in range(n_veh)]
		dy_list = [(1./self.T)*y_list[i].derivative() for i in range(n_veh)]
		ddx_list = [(1./(self.T**2))*x_list[i].derivative(2) for i in range(n_veh)]
		ddy_list = [(1./(self.T**2))*y_list[i].derivative(2) for i in range(n_veh)]

		return x_list, y_list,dx_list,dy_list,ddx_list,ddy_list


	def plot_sol(self,veh_id_list):
		# Plotting of the different trajectories. 
		colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'orange','darkgreen']
		n_veh = len(veh_id_list)
		n_animframes = self.n_animframes
		t = self.T
		time = np.linspace(0, t, n_animframes)
		fig = plt.figure()
		ax = plt.axes(xlim=(0, self.environment.width), ylim=(0, self.environment.height))
		for i in veh_id_list:
			x,y=list(map(lambda x:x[0],self.waypointslist[i])),list(map(lambda x:x[1],self.waypointslist[i]))
			plt.plot(x,y,colors[i%8])

		obstx,obsty = np.append(self.obst_vert[0], self.obst_vert[0, 0]),np.append(self.obst_vert[1], self.obst_vert[1, 0]) #append first node at the end such that 4 lines will be drawn
		obst = np.vstack((obstx,obsty)) 
		lines = []		
		for i in veh_id_list:
			lines.append(plt.plot([], [], colors[i%8])[0])

		for i in range(self.n_racks):
			lines.append(plt.plot(obst[0, :] + self.pos_obst[i][0], obst[1, :] + self.pos_obst[i][1], 'k')[0])
		lines.append(plt.plot([0,self.environment.dz_x], [self.environment.dz_y,self.environment.dz_y], 'k')[0])


		def animate(k): #make elements move
			s = np.linspace(0, 1-1./48, 48)
			for i in veh_id_list:
				try:
					lines[i].set_data(self.route_sampled_x[i][k] + self.vehicles[i].get_rad()*np.cos(s*2*np.pi), self.route_sampled_y[i][k] + self.vehicles[i].get_rad()*np.sin(s*2*np.pi))
		 		except:
		 			pass
		 	return lines
		for i in veh_id_list:
			plt.plot(self.route_sampled_x[i],self.route_sampled_y[i],colors[i%8],dashes=[2, 2])

		fps = 30 # frames per second
		anim = animation.FuncAnimation(fig, animate, frames=n_animframes, interval=1000/fps, blit=True)
		# duration
		# saved : frames * (1 / fps)
		# showed: frames * interval / 1000


		if self.show:
			plt.show()

		#Code for saving the movie
		inp = int(raw_input("Enter 1 if you want to save:"))
		if inp == 1:
			self.writemovie = True
		if self.writemovie:
			print("Saving... ")
			anim.save(self.moviename+'.gif',fps =fps,writer='imagemagick')


	def provide_waypoints(self,veh_id_list):
		# Global path planning phase of the algorithm using the AStarPlanner. 
		# the output is a list of waypointlists (each vehicle gets a waypointlist)
		# The latter is used in a second step to identify the surrounding corridors. 
		waypoints_list = []
		for veh in self.vehicles:
			if veh.get_id() in veh_id_list:
				planner = AStarPlanner(self.environment,[self.n_cellsx,self.n_cellsy],veh.get_start_pos(),veh.get_target())
				waypoints = planner.get_path()
				self.waypointslist[veh.get_id()] = waypoints

		return 

	def from_waypoints_to_corridors(self,waypoints):
		# The purpose of this function is to boil down the sequence of waypoints to 
		# a sequence of individual corridors, or frames. 
		# Because some waypoints may lay on crossings of frames, caution is needed during the 
		# identifications. If everything went as planned, this method should work independent 
		# of the frame format, (width and height of the rectangular corridor frames) used. 
		corridors = self.environment.get_corridors_around_points(waypoints)
		# Eliminate possibility of not needed corridor if first waypoint lays on aisle crossing
		lookindex = 1
		intersect = set(corridors[0]).intersection(set(corridors[lookindex]))
		while len(intersect) != 1:
			lookindex += 1
			intersect = intersect.intersection(set(corridors[lookindex]))
		if len(intersect) == 1:
			del corridors[:lookindex]
			for i in range(lookindex):
				corridors.insert(0,list(intersect))
		else:
			print("Error, cannot find corridor")
		lookindex = 1
		intersect = set(corridors[-1]).intersection(set(corridors[-1-lookindex]))
		while len(intersect) != 1:
			lookindex += 1
			intersect = intersect.intersection(set(corridors[-1-lookindex]))
		if len(intersect) == 1:
			del corridors[-lookindex:]
			for i in range(lookindex):
				corridors.append(list(intersect))
		else:
			print("Error, cannot find corridor")


		for i, item in enumerate(corridors):
			if len(item) > 1:
				lookindex = 1
				while (item == corridors[i+lookindex]) :#and (i+lookindex != len(corridors)):
					lookindex += 1 
				if corridors[i-1] == corridors[i+lookindex]:
					# crossing is not taken
					for j in range(i,i+lookindex):
						corridors[j] = corridors[i-1]

		return corridors


	def set_all_corridor_parameters(self,veh_id_list):
		# Set the parameters for all the vehicles, and make a combined paramater set 'allp'
		allp = []
		for vehicle_nb in veh_id_list:
			allp.append(self.set_corridor_parameters(vehicle_nb))
		return allp
		
	def set_corridor_parameters(self,vehicle_nb):
		# This function's purpose is to set for the vehicle corresponding with vehicle_nb a corridor parameter set list. 
		# This parameters include the dimensions of the corridor, to which the vehicle is therefore bounded. 
		corridors = self.from_waypoints_to_corridors(self.waypointslist[vehicle_nb]) 
		crossing_index = [] 
		x_min_list = []
		y_min_list = []
		x_max_list = []
		y_max_list = []
		corr_types = []
		corr_ids = []
		last_reachable_waypoints = []
		start = []
		end = []

		for i in range(len(corridors)):
			if len(corridors[i]) > 1 and len(corridors[i+1]) == 1: #this means a crossing is reached, and a bend will be made. Remember that frames that are not needed are already filtered out / assumed not to appear 
				crossing_index.append(i)

		start.append(self.vehicles[vehicle_nb].get_start_pos()) 
		[x_min,y_min,x_max,y_max] = corridors[0][0].get_limits() #corridors is list of lists
		x_min_list.append(x_min)
		y_min_list.append(y_min)
		x_max_list.append(x_max)
		y_max_list.append(y_max)
		corr_types.append(corridors[0][0].get_type())
		corr_ids.append(corridors[0][0].get_id())
		for i,cr_index in enumerate(crossing_index):
			end.append(self.waypointslist[vehicle_nb][cr_index])
			corr_types.append(corridors[cr_index+1][0].get_type())
			corr_ids.append(corridors[cr_index+1][0].get_id())
			start.append(end[-1])
			[x_min,y_min,x_max,y_max] = corridors[cr_index+1][0].get_limits() #From from_waypoints_to_corridors garantees just 1 corridor waypoint after crossing
			x_min_list.append(x_min)
			y_min_list.append(y_min)
			x_max_list.append(x_max)
			y_max_list.append(y_max)
		end.append(self.vehicles[vehicle_nb].get_target())
		

		parameters = start,end,x_min_list,y_min_list,x_max_list,y_max_list,corr_types,corr_ids
		
		self.decouple_parameters(parameters,vehicle_nb)
		return


	def decouple_parameters(self,parameters,vehicle_nb):
		# Converts the given parameteters of vehicle with id vehicle_nb from 
		# [start1,start2,start3], [end1,end2,end3]
		# to 
		# [[start1,end1],[start2,end2],[start3,end3]] like 
		# Additionally, extra parameters are determined and added to the parameters
		# e.g. the treshold values needed to determine when a switch is made from multiframe F1-F2 towards
		# multiframe F2-F3 and the position of the rack corners, for the pivoting separating hyperplane. 
		# These parameters are thus dependent on the type of multi-frame e.g. the kind of turn which is made.
		# Therefore a subdivision in different turns (see further on) by means of a simple graphic
		# representation is used.

		start,end,x_min_list,y_min_list,x_max_list,y_max_list,corr_types,corr_ids  = parameters

		infinity = self.n_animframes -1
		offset = 0.001

		allparam = []

		# F stands for final 
		F_x_start_list =[]
		F_y_start_list =[]
		F_x_end_list =[]
		F_y_end_list =[]
		F_x_min_list =[]
		F_y_min_list =[]
		F_x_max_list =[]
		F_y_max_list =[]
		F_x_hp_list =[]
		F_y_hp_list =[]
		F_x_hp2_list =[]
		F_y_hp2_list =[]
		F_x_hp3_list =[]
		F_y_hp3_list =[]

		F_corrids_list = []

		F_x_mid_list =[]
		F_y_mid_list =[]
		F_treshold_x_list = []
		F_treshold_y_list = []


		if len(start) == 1:
			F_x_start_list.append(start[0][0])
			F_y_start_list.append(start[0][1])
			F_x_end_list.append(end[0][0])
			F_y_end_list.append(end[0][1])
			
			F_x_min_list.append(x_min_list[0] )#+ self.vehicles[vehicle_nb].get_rad())
			F_y_min_list.append(y_min_list[0] )#+ self.vehicles[vehicle_nb].get_rad())
			F_x_max_list.append(x_max_list[0] )#- self.vehicles[vehicle_nb].get_rad())
			F_y_max_list.append(y_max_list[0] )#- self.vehicles[vehicle_nb].get_rad())
			F_x_hp_list.append(infinity)
			F_y_hp_list.append(infinity)
			F_x_hp2_list.append(infinity)
			F_y_hp2_list.append(infinity)
			F_treshold_x_list.append(end[0][0])
			F_treshold_y_list.append(end[0][1])
			F_corrids_list.append([corr_ids[0]])


		for i in range(len(start)-1):
			xstart, ystart, xend, yend = start[i][0],start[i][1],end[i+1][0],end[i+1][1]
			if (corr_types[i] == 'horizontal' and corr_types[i+1] == 'vertical'): #type of bend - h->v
				if xstart < xend:
					if ystart < yend:
						# Case of 
						#		^
						#		|
						#      o|
						#  -----
						xmin = -infinity
						ymin = y_min_list[i] + self.vehicles[vehicle_nb].get_rad()
						xmax = x_max_list[i+1] - self.vehicles[vehicle_nb].get_rad()
						ymax = +infinity
						xhp,yhp = x_min_list[i+1],y_max_list[i]
						xhp2,yhp2 = -1,+1 	#xhp2,yhp2 contain the signs used in separating hyperplane constraint. These are 
											#dependent on the kind of bend which is taken. To fully understand starting and end values of 
											# x and y should be considered in combination with a look at the constraint equation. 
						xmid,ymid = xhp + self.environment.aisle_x/2 , yhp - self.vehicles[vehicle_nb].get_rad()
						# treshold values indicate the bound when the vehicle enters the following frame combo (L shape)
						tresholdx = x_min_list[i+1]+self.vehicles[vehicle_nb].get_rad()
						tresholdy = infinity#y_max_list[i]

					else:
						# Case of 
						# -------
						#      o|
						#       |
						#       v
						xmin = -infinity
						ymin = -infinity
						xmax = x_max_list[i+1] - self.vehicles[vehicle_nb].get_rad()
						ymax = y_max_list[i] - self.vehicles[vehicle_nb].get_rad()
						xhp,yhp = x_min_list[i+1],y_min_list[i]
						xhp2,yhp2 = -1,-1
						xmid,ymid = xhp + self.environment.aisle_x/2 , yhp + self.vehicles[vehicle_nb].get_rad()
						tresholdx = x_min_list[i+1]+self.vehicles[vehicle_nb].get_rad()
						tresholdy = infinity

				else:
					if ystart < yend:
						# Case of 
						#		^
						#		|
						#       |o
						#        -------
						xmin = x_min_list[i+1] + self.vehicles[vehicle_nb].get_rad()
						ymin = y_min_list[i] + self.vehicles[vehicle_nb].get_rad()
						xmax = +infinity
						ymax = +infinity
						xhp,yhp = x_max_list[i+1],y_max_list[i]
						xhp2,yhp2 = +1,+1
						xmid,ymid = xhp - self.environment.aisle_x/2 , yhp - self.vehicles[vehicle_nb].get_rad()
						tresholdx = x_max_list[i+1]-self.vehicles[vehicle_nb].get_rad()
						tresholdy = infinity

					else:
						# Case of 
						#        -------
						#       |o
						#       |
						#       v
						xmin = x_min_list[i+1] + self.vehicles[vehicle_nb].get_rad()
						ymin = -infinity
						xmax = +infinity
						ymax = y_max_list[i]  - self.vehicles[vehicle_nb].get_rad()
						xhp,yhp = x_max_list[i+1],y_min_list[i]
						xhp2,yhp2 = +1,-1
						xmid,ymid = xhp - self.environment.aisle_x/2 , yhp + self.vehicles[vehicle_nb].get_rad()
						tresholdx = x_max_list[i+1]-self.vehicles[vehicle_nb].get_rad()
						tresholdy = infinity


			elif (corr_types[i] == 'vertical' and corr_types[i+1] == 'horizontal'): #type of bend | v->h
				if xstart > xend:
					if ystart > yend:
						# Case of 
						#		|
						#		|
						#      o|
						#  <----
						xmin = -infinity
						ymin = y_min_list[i+1] + self.vehicles[vehicle_nb].get_rad()
						xmax = x_max_list[i]  - self.vehicles[vehicle_nb].get_rad()
						ymax = +infinity
						xhp,yhp = x_min_list[i],y_max_list[i+1]
						xhp2,yhp2 = -1,+1
						xmid,ymid = xhp + self.vehicles[vehicle_nb].get_rad() + offset , yhp - self.environment.aisle_y/2
						tresholdx = infinity
						tresholdy = y_max_list[i+1]-self.vehicles[vehicle_nb].get_rad()


					else:
						# Case of 
						# <------
						#      o|
						#       |
						#       |
						xmin = -infinity
						ymin = -infinity
						xmax = x_max_list[i]  - self.vehicles[vehicle_nb].get_rad()
						ymax = y_max_list[i+1] - self.vehicles[vehicle_nb].get_rad()
						xhp,yhp = x_min_list[i],y_min_list[i+1]
						xhp2,yhp2 = -1,-1
						xmid,ymid = xhp + self.vehicles[vehicle_nb].get_rad() + offset , yhp + self.environment.aisle_y/2
						tresholdx = infinity
						tresholdy = y_min_list[i+1]+self.vehicles[vehicle_nb].get_rad()
				else:
					if ystart > yend:
						# Case of 
						#		|
						#		|
						#       |o
						#        ------>
						xmin = x_min_list[i] + self.vehicles[vehicle_nb].get_rad()
						ymin = y_min_list[i+1] + self.vehicles[vehicle_nb].get_rad()
						xmax = +infinity
						ymax = +infinity
						xhp,yhp = x_max_list[i],y_max_list[i+1]
						xhp2,yhp2 = +1,+1
						xmid,ymid = xhp - self.vehicles[vehicle_nb].get_rad() - offset , yhp - self.environment.aisle_y/2
						tresholdx = infinity
						tresholdy = y_max_list[i+1]-self.vehicles[vehicle_nb].get_rad()

					else:
						# Case of 
						#        ------>
						#       |o
						#       |
						#       |
						xmin = x_min_list[i] + self.vehicles[vehicle_nb].get_rad()
						ymin = -infinity
						xmax = +infinity
						ymax = y_max_list[i+1] - self.vehicles[vehicle_nb].get_rad()
						xhp,yhp = x_max_list[i],y_min_list[i+1]
						xhp2,yhp2 = +1,-1 
						xmid,ymid = xhp - self.vehicles[vehicle_nb].get_rad() - offset , yhp + self.environment.aisle_y/2
						tresholdx = infinity
						tresholdy = y_min_list[i+1]+self.vehicles[vehicle_nb].get_rad()

			elif (corr_types[i] == 'horizontal' and corr_types[i+1] == 'horizontal'):
				if x_min_list[i] < x_min_list[i+1]: #left to right
					xmin = x_min_list[i] + self.vehicles[vehicle_nb].get_rad()
					ymin = y_min_list[i] + self.vehicles[vehicle_nb].get_rad()
					xmax = x_max_list[i+1]
					ymax = y_max_list[i+1] - self.vehicles[vehicle_nb].get_rad()
					xhp,yhp = infinity,infinity
					xhp2,yhp2 = +1,-1 
					xmid,ymid = x_max_list[i],y_min_list[i]+self.environment.aisle_y/2
					tresholdx = x_min_list[i+1] + self.vehicles[vehicle_nb].get_rad()
					tresholdy = infinity
				else : #right to left
					xmin = x_min_list[i+1] + self.vehicles[vehicle_nb].get_rad()
					ymin = y_min_list[i] + self.vehicles[vehicle_nb].get_rad()
					xmax = x_max_list[i]
					ymax = y_max_list[i+1] - self.vehicles[vehicle_nb].get_rad()
					xhp,yhp = infinity,infinity
					xhp2,yhp2 = +1,-1 
					xmid,ymid = x_min_list[i],y_min_list[i]+self.environment.aisle_y/2
					tresholdx = x_max_list[i+1] - self.vehicles[vehicle_nb].get_rad()
					tresholdy = infinity

			elif (corr_types[i] == 'vertical' and corr_types[i+1] == 'vertical'):
				if y_min_list[i] < y_min_list[i+1]: # upwards 
					xmin = x_min_list[i] + self.vehicles[vehicle_nb].get_rad()
					ymin = y_min_list[i]
					xmax = x_max_list[i] - self.vehicles[vehicle_nb].get_rad()
					ymax = y_max_list[i+1] 
					xhp,yhp = +infinity,+infinity
					xhp2,yhp2 = +1,-1
					xmid,ymid = x_min_list[i] + self.environment.aisle_x/2,y_max_list[i]
					tresholdx = infinity
					tresholdy = y_min_list[i+1] +self.vehicles[vehicle_nb].get_rad()
				else: # downwards 
					xmin = x_min_list[i] + self.vehicles[vehicle_nb].get_rad()
					ymin = y_min_list[i+1]
					xmax = x_max_list[i] - self.vehicles[vehicle_nb].get_rad()
					ymax = y_max_list[i] 
					xhp,yhp = infinity,infinity
					xhp2,yhp2 = +1,-1 
					xmid,ymid = x_min_list[i] + self.environment.aisle_x/2,y_min_list[i]
					tresholdx = infinity
					tresholdy = y_max_list[i+1] - self.vehicles[vehicle_nb].get_rad()

			
			else:
				print("ERROR: Faults in corridor types, a horizontal corridor is not followed by a vertical one.")
				print(corr_types[i],corr_types[i+1])
			
			corr_combo = [corr_ids[i],corr_ids[i+1]]

			F_x_start_list.append(xstart)
			F_y_start_list.append(ystart)
			F_x_end_list.append(xend)
			F_y_end_list.append(yend)
			F_x_min_list.append(xmin)
			F_y_min_list.append(ymin)
			F_x_max_list.append(xmax)
			F_y_max_list.append(ymax)
			F_x_hp_list.append(xhp)
			F_y_hp_list.append(yhp)
			F_x_hp2_list.append(xhp2)
			F_y_hp2_list.append(yhp2)			
			F_x_mid_list.append(xmid)
			F_y_mid_list.append(ymid)
			F_treshold_x_list.append(tresholdx)
			F_treshold_y_list.append(tresholdy)
			F_corrids_list.append(corr_combo)

		# TODO: Transform allparam to object or dict 	
		for i in range(len(F_x_start_list)-1): 
			p = Parameters(F_x_start_list[i],F_y_start_list[i],F_x_mid_list[i+1],F_y_mid_list[i+1],F_x_min_list[i],F_y_min_list[i],F_x_max_list[i],F_y_max_list[i],F_x_hp_list[i],F_y_hp_list[i],F_x_hp2_list[i],F_y_hp2_list[i],F_treshold_x_list[i],F_treshold_y_list[i],F_corrids_list[i])
			self.vehicles[vehicle_nb].add_parameters(p)

		i = len(F_x_start_list)-1
		p = Parameters(F_x_start_list[i],F_y_start_list[i],F_x_end_list[i],F_y_end_list[i],F_x_min_list[i],F_y_min_list[i],F_x_max_list[i],F_y_max_list[i],F_x_hp_list[i],F_y_hp_list[i],F_x_hp2_list[i],F_y_hp2_list[i],F_treshold_x_list[i],F_treshold_y_list[i],F_corrids_list[i])
		self.vehicles[vehicle_nb].add_parameters(p)

		


		return 




	


