"""
This file is part of the master thesis of Michiel De Deken, MSc. in Engineering: Logistics & Traffic @ KU Leuven: 

" Free motion planning of multiple AGVs using a spline based-approach
--- A multi-frame technique applied to a warehouse environment --- "

June 2018


Author: Michiel De Deken

"""
class Parameters(object):
	# parameter class of a vehicle

	def __init__(self,startx,starty,endx,endy,xmin,ymin,xmax,ymax,xhp,yhp,xhp2,yhp2,treshold_x,treshold_y,corrids):
		self.startx = startx
		self.starty = starty
		self.endx = endx
		self.endy = endy
		self.xmin = xmin
		self.xmax = xmax
		self.ymin = ymin
		self.ymax = ymax
		self.xhp = xhp
		self.yhp = yhp
		self.xhp2 = xhp2
		self.yhp2 = yhp2
		self.treshold_x = treshold_x
		self.treshold_y = treshold_y
		self.corrids = corrids


	def convert_to_problem_par(self):
		par = [self.startx,self.starty,self.endx,self.endy,self.xmin,self.ymin,self.xmax,self.ymax,self.xhp,self.yhp,self.xhp2,self.yhp2,self.treshold_x,self.treshold_y,self.corr_v0x,self.corr_v0y,self.maxVel,self.maxAcc,self.rad]
		par.extend(self.overlaplist)
		return par

	def set_initial_velocity(self,corr_v0x,corr_v0y):
		self.corr_v0x = corr_v0x
		self.corr_v0y = corr_v0y

	def set_maxVelAccRad(self,maxVel,maxAcc,rad):
		self.maxVel = maxVel
		self.maxAcc = maxAcc
		self.rad = rad

	def set_start_pos(self,new_startx,new_starty):
		self.startx = new_startx
		self.starty = new_starty

	def get_start_pos(self):
		return self.startx,self.starty

	def set_overlaplist(self,overlaplist):
		self.overlaplist = overlaplist