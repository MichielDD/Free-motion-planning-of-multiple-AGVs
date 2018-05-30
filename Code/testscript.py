"""
This file is part of the master thesis of Michiel De Deken, MSc. in Engineering: Logistics & Traffic @ KU Leuven: 

" Free motion planning of multiple AGVs using a spline based-approach
--- A multi-frame technique applied to a warehouse environment --- "

June 2018


Author: Michiel De Deken

"""


from problem import *
from wh_environment import *
from vehicle import *
from globalplanner import *
from random import shuffle,randint

nbofvehlist = [10,10,5,10]#,25]
row = [2,7,7]#,7,8]
column = [2,6,7]#,7,8]

for k in range(0,1):
	


	### Warehouse parameters ###
	rows = row[k]
	columns = column[k]
	aisle_x = 2.
	aisle_y = 2.
	rack_y= 4.0
	rack_x = 2.0 
	buffer_x = aisle_x
	buffer_y = aisle_y
	dz_y = 1.5
	width = 2*buffer_x + columns * rack_x + (columns - 1)*aisle_x
	height = dz_y + 2*buffer_y + rows * rack_y + (rows - 1)*aisle_y
	dz_x = width



	### Vehicle parameters ### 

	max_vel = 3.0
	max_acc = 3.0
	rad = 0.2
	nbofveh = nbofvehlist[k]


	### problem parameters for Global path planning A* ### 
	n_cellsx = 80
	n_cellsy = n_cellsx

	moviename = 'anim_warehouse'+str(nbofveh)+'-r'+str(rows)+'-c'+str(columns)+'Generic_SOLVED_2205'
	t = time.time()

	# Initialize Environment
	w = Warehouse(width=width,height=height,obstacles=None,rows=rows,columns=columns,aisle_x=aisle_x,aisle_y=aisle_y,rack_x=rack_x,rack_y=rack_y,buffer_x=buffer_x,buffer_y=buffer_y,dz_x=dz_x,dz_y=dz_y,n_cellsx=n_cellsx,n_cellsy=n_cellsy)
	vehicles = []


	# Set up start and end locations (Randomly)
	centres = (w.corridor_centres)
	shuffle(centres)
	corridor_centres = centres

	l = len(corridor_centres)
	init_pos_list = []
	target_list = []

	print('Start init pos calc')
	# while len(init_pos_list) < nbofveh:
	# 	s = randint(0,l-1)
	# 	#print(s)
	# 	if corridor_centres[s] not in init_pos_list:
	# 		init_pos_list.append(corridor_centres[s])

	vinit = [[i, j] for i in np.linspace(buffer_x*2.5 ,width-buffer_x*2.5,columns-1) for j in np.linspace(dz_y+ buffer_y,height - buffer_y,nbofveh)]
	hinit = [[i, j] for i in np.linspace(buffer_x*0.89,width-buffer_x*0.89,nbofveh) for j in np.linspace(dz_y+ buffer_y+ aisle_y/2+rack_y,height -buffer_y-rack_y-aisle_y/2,rows-1)]
	init_pos_list = vinit + hinit 
	shuffle(init_pos_list)
	print('End init pos calc')

	targets_left = [[rad*2,j] for j in np.linspace(dz_y+2*rad,height-2*rad,nbofveh/3)]
	targets_up = [[i, height-2*rad] for i in np.linspace(rad*2 + 2 ,width-2 -2*rad,nbofveh/3)]
	targets_right = [[width-rad*2,j] for j in np.linspace(dz_y+2*rad,height-2*rad,nbofveh-2*int(nbofveh/3))]
	target_list = targets_left+targets_right+targets_up
	shuffle(target_list)
	#init_pos_list = target_list
	#shuffle(init_pos_list)
	# Test purpose : Small network test
	#init_pos_list = [[8.0, 8.5],[2.7,8.]]
	#target_list = [[9.6, 1.8999999999999999],[2.5,2]]
	#init_pos_list= [[7.0, 2.5],[3.0, 8.5],[9.0, 11.5],[1.0, 5.5]]
	#target_list = [[3.0, 8.5], [9.0, 11.5],[1.0, 5.5],[7.0, 2.5]]
	# init_pos_list= [[9.0, 11.5],[3.0, 8.5],]
	# target_list = [[1.0, 5.5], [9.0, 11.5]]


	# Initialize vehicles
	print("Positions")
	for i in range(nbofveh):	
		veh = Vehicle(i,init_pos_list[i],target_list[i],max_vel,max_acc,rad)
		print(init_pos_list[i],target_list[i])
		vehicles.append(veh)

	# Solve Problem
	prob = Problem(w,vehicles,n_cellsx,n_cellsy,moviename,show=True,writemovie=False,timehorizon=30.0)



