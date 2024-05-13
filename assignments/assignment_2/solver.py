import sys
import math
import numpy as np
import random
import time
from support.problem_spec import ProblemSpec
from support.robot_config import make_robot_config_from_ee1, make_robot_config_from_ee2
from support.angle import Angle

class GraphNode:
	"""
	Class representing a node in the state graph.
	"""
	def __init__(self, spec, config):
		"""
		Create a new graph node object for the given config.
		:param spec: ProblemSpec object
		:param config: the RobotConfig object to be stored in this node
		"""
		self.spec = spec
		self.config = config
		self.neighbors = []
		if config.ee1_grappled:
			self.grapple_point = config.points[0]
			self.grapple_angle = config.ee1_angles[0]
		else:
			self.grapple_point = config.points[-1]
			self.grapple_angle = config.ee2_angles[0]

		for i in range(len(spec.grapple_points)):
			if point_is_close(self.grapple_point, spec.grapple_points[i], spec.TOLERANCE):
				self.grapple_index = i
				break
		
	def get_successors(self):
		return self.neighbors

def cb_distance(coordA, coordB):
	""" Return city-block distance between 2 points
	"""
	xA, yA = coordA
	xB, yB = coordB
	return (abs(xA-xB) + abs(yA-yB))

def eu_distance(pointA, pointB):
	""" Return euclidean distance between 2 points
	"""
	xA, yA = pointA
	xB, yB = pointB
	return math.sqrt((xA-xB)*(xA-xB) + (yA-yB)*(yA-yB))
	
def angle_between(pointA, pointB, pointC):
	""" Return a positive angle between 3 points
	"""
	u = (pointB[0]-pointA[0], pointB[1]-pointA[1])
	v = (pointC[0]-pointB[0], pointC[1]-pointB[1])
	norm_u = math.sqrt(u[0]*u[0]+u[1]*u[1])
	norm_v = math.sqrt(v[0]*v[0]+v[1]*v[1])
	cross = u[0]*v[1]-u[1]*v[0]
	theta = math.asin(cross/(norm_u*norm_v))
	return theta

"""Test for colision
"""	
def test_environment_bounds(node):
	# return true for pass, false for fail
	config = node.config
	for x, y in config.points:
		if not 0.0 <= x <= 1.0:
			return False
		if not 0.0 <= y <= 1.0:
			return False
	return True	

def point_is_close(pointA, pointB, tolerance):
	return abs(pointB[0] - pointA[0]) + abs(pointB[1] - pointA[1]) < tolerance	

def intersected(A,B,C,D, tolerance):
	AB_cross = ((C[0]-A[0])*(B[1]-A[1]) - (C[1]-A[1])*(B[0]-A[0]))*((D[0]-A[0])*(B[1]-A[1]) - (D[1]-A[1])*(B[0]-A[0]))
	CD_cross = ((A[0]-C[0])*(D[1]-C[1]) - (A[1]-C[1])*(D[0]-C[0]))*((B[0]-C[0])*(D[1]-C[1]) - (B[1]-C[1])*(D[0]-C[0]))
	if AB_cross < 0 and CD_cross < 0:
		return True
	return False

def test_collision(node):
	# return true for pass, false for fail
	num_segments = node.spec.num_segments
	config = node.config
	config_points = config.points
	spec = node.spec
	obstacles = spec.obstacles
	if not config.ee1_grappled:
		config_points = list(p for p in reversed(config_points))
	
	for i in range(num_segments):
		for k in range(num_segments):	
			if k not in (i-1, i, i+1):
				if intersected(config_points[i], config_points[i+1], config_points[k], config_points[(k+1)], spec.TOLERANCE):
					#print("k = (%d,%d), i- = (%d,%d), i+ = (%d, %d)" %(k, k+1, i-1, i, i, i+1))
					return False
		
		for obs in obstacles:
			obs_points = obs.corners
			for j in range(4):
				if intersected(config_points[i], config_points[i+1], obs_points[j], obs_points[(j+1)%4], spec.TOLERANCE):
					return False
	return True

def test_config_distance(c1, c2, spec):
	# return maximum distance between 2 configurations
	if c1.ee1_grappled != c2.ee1_grappled:
		return False
	for i in range(spec.num_segments):
		if c1.ee1_grappled:
			if abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians()) > spec.PRIMITIVE_STEP + spec.TOLERANCE:
				return False
		else:
			if abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians()) > spec.PRIMITIVE_STEP + spec.TOLERANCE:
				return False
	
	for i in range(spec.num_segments):
		if abs(c2.lengths[i] - c1.lengths[i]) > spec.PRIMITIVE_STEP + spec.TOLERANCE:
			return False
	return True
	
def config_random(spec, grapple_index, ee1_grapple_points, ee2_grapple_points, ee1_grappled=False, min_length=False):
	""" Randomly generate a robot config
	"""
	num_segments = spec.num_segments
	min_lengths, max_lengths = spec.min_lengths, spec.max_lengths
	grapple_points = spec.grapple_points
	angles = []
	lengths = []
	grappled_eex, grappled_eey = grapple_points[grapple_index]
	angles = [Angle(degrees=random.uniform(-180, 180))]
	if min_length:
		lengths = [min_lengths[0]] #[random.uniform(min_lengths[0], (min_lengths[0]+max_lengths[0])/2)]
	else: 
		lengths = [random.uniform(min_lengths[0], max_lengths[0])]
		
	for i in range(1,num_segments):
		angles.append(Angle(degrees=random.uniform(-165, 165)))
		if min_length:
			lengths.append(min_lengths[i]) #random.uniform(min_lengths[i], (min_lengths[i]+max_lengths[i])/2))
		else:
			lengths.append(random.uniform(min_lengths[i], max_lengths[i]))
	if (ee1_grappled) or (grapple_index in ee1_grapple_points):	
		return make_robot_config_from_ee1(grappled_eex, grappled_eey, angles, lengths, ee1_grappled=True)
	elif (grapple_index in ee2_grapple_points):
		return make_robot_config_from_ee2(grappled_eex, grappled_eey, angles, lengths, ee2_grappled=True)
	else:
		if random.random() < 0.5:
			return make_robot_config_from_ee1(grappled_eex, grappled_eey, angles, lengths, ee1_grappled=True)
		else:
			return make_robot_config_from_ee2(grappled_eex, grappled_eey, angles, lengths, ee2_grappled=True)
			
def mid_node(nodeA, nodeB):
	""" Generate the middle node between nodeA and nodeB
	"""
	configA = nodeA.config
	configB = nodeB.config
	spec = nodeA.spec

	if configA.ee1_grappled:
		anglesA = configA.ee1_angles
		anglesB = configB.ee1_angles
	else:
		anglesA = configA.ee2_angles
		anglesB = configB.ee2_angles
		
	angles = [Angle(degrees=(anglesA[i].in_degrees()+anglesB[i].in_degrees())/2) for i in range(len(anglesA))] 
	lengthsA = configA.lengths 
	lengthsB = configB.lengths 
	lengths = [(lengthsA[i] + lengthsB[i])/2 for i in range(len(lengthsA))]
	grappled_eex, grappled_eey = nodeA.grapple_point
	if configA.ee1_grappled:
		configAB = make_robot_config_from_ee1(grappled_eex, grappled_eey, angles, lengths, ee1_grappled=True)
	else:
		configAB = make_robot_config_from_ee2(grappled_eex, grappled_eey, angles, lengths, ee2_grappled=True)		
	return GraphNode(spec, configAB)			

def possible_path(nodeA, nodeB):
	""" Check if the grapple points are valide between nodeA and nodeB
	"""
	spec = nodeA.spec
	configA = nodeA.config
	configB = nodeB.config
	if (configA.ee1_grappled != configB.ee1_grappled) or (nodeA.grapple_index != nodeB.grapple_index):	
		return False
	return True

def nodes_path(nodeA, nodeB):
	""" Return a path between nodeA and nodeB, return None if there is no path
	"""
	spec = nodeA.spec
	configA = nodeA.config
	configB = nodeB.config
	num_segments = spec.num_segments	
	if test_config_distance(configA, configB, spec):
		return [nodeA, nodeB]		
	midAB = mid_node(nodeA, nodeB)
	if (not test_collision(midAB)) or (not test_environment_bounds(midAB)):
		#print("Colided Mid Config")
		return None
	try:
		path = nodes_path(nodeA, midAB) + nodes_path(midAB, nodeB)
		return path
	except Exception:
		#print("Invalid path")
		return None
			
def grapple_node_create(node, grapple_point):
	""" Return a pair of nodes that is built from the (closed) node and the grapple point
		Return None if the node is too far from the grapple point
	"""
	config = node.config
	spec = node.spec	
	points = config.points
	min_lengths, max_lengths = spec.min_lengths, spec.max_lengths
	
	last_length = eu_distance(points[-2], grapple_point)
	if (min_lengths[-1] <= last_length) and (last_length <= max_lengths[-1]):
		last_angle = Angle(radians=angle_between(points[-3],points[-2],grapple_point))
		if (-165 <= last_angle.in_degrees()) and (last_angle.in_degrees() <= 165):	
			gnodeA_eex, gnodeA_eey = points[0]
			gnodeB_eex, gnodeB_eey = grapple_point
			gnodeA_angles = [a for a in config.ee1_angles[0:-1]] + [last_angle]
			lengths = [l for l in config.lengths[0:-1]] + [last_length]
			
			gnodeA_config = make_robot_config_from_ee1(gnodeA_eex, gnodeA_eey, gnodeA_angles, lengths, ee1_grappled=True)
			gnodeA = GraphNode(spec, gnodeA_config)
			
			gnodeB_angles = gnodeA_config.ee2_angles
			gnodeB_config = make_robot_config_from_ee2(gnodeB_eex, gnodeB_eey, gnodeB_angles, lengths, ee2_grappled=True)			
			gnodeB = GraphNode(spec, gnodeB_config)
			
			if not point_is_close(gnodeA.config.points[-1], grapple_point, spec.TOLERANCE):
				return None			
			if (not test_collision(gnodeA)) or (not test_environment_bounds(gnodeA)):
				#print("colided grapple nodes A")
				return None
			if (not test_collision(gnodeB)) or (not test_environment_bounds(gnodeB)):
				#print("colided grapple nodes B")
				return None					
			return (gnodeA, gnodeB)
	return None
	
def possible_grapple(index_count, intermediate_grapples, index, poss=None, all_poss=None):
	""" Return all possible orders of movement from one grapple to another
	"""
	if poss is None:
		poss = list((0*i-1) for i in range(index_count))
	if index == index_count:
		all_poss.append(list(poss))
		return
	else:
		for i in range(len(intermediate_grapples)):
			poss[index] = intermediate_grapples[i]
			new_grapples_list = list(intermediate_grapples[ind] for ind in range(len(intermediate_grapples)) if ind != i)
			possible_grapple(index_count, new_grapples_list, index+1, poss=poss, all_poss=all_poss)
	return
	
def find_grapple_path(init_node, goal_node, spec, ee1_grapple_points, ee2_grapple_points):
	""" Return a list of possible grapple nodes that will be the milestones for the final paths
	"""
	if len(spec.grapple_points) == 1:
		return [init_node, goal_node]
	num_grapples = len(spec.grapple_points)
	grapple_found = False
	
	init_index = init_node.grapple_index
	goal_index = goal_node.grapple_index
	
	intermediate_grapples = list(ind for ind in range(num_grapples) if (ind != init_node.grapple_index) and (ind != goal_node.grapple_index))
	possible_grapple_path = []
	if num_grapples == 2:
		possible_grapple_path = [[init_index, goal_index]]
	else:
		if init_node.config.ee1_grappled != goal_node.config.ee1_grappled:
			index_count = 2
		else:
			index_count = 1

		while index_count < num_grapples-1:	
			poss_paths = []
			possible_grapple(index_count, intermediate_grapples, 0, poss=None, all_poss=poss_paths)
			for path in poss_paths:
				path.insert(0,init_index)
				path.append(goal_index)
				
			possible_grapple_path = possible_grapple_path + poss_paths
			index_count += 2

	count = 0
	while (not grapple_found):
		poss_index = count%len(possible_grapple_path)
		count += 1
		poss_path = possible_grapple_path[poss_index]
		ee1_list = []
		ee2_list = []

		if init_node.config.ee1_grappled:
			for i in range(len(poss_path)):
				if i%2==0:
					ee1_list.append(poss_path[i])
				else: 
					ee2_list.append(poss_path[i])
		else:
			for i in range(len(poss_path)):
				if i%2==0:
					ee2_list.append(poss_path[i])
				else: ee1_list.append(poss_path[i])
					
		grapple_neighbors = {init_node: [], goal_node: []}
		grapple_nodes = [init_node, goal_node]
		for index in range(len(poss_path)-1):		
			grapple_nodeAB = grapple_connect(poss_path[index], poss_path[index+1], spec, ee1_list, ee2_list)

			if grapple_nodeAB is not None:	
				(grapple_nodeA, grapple_nodeB) = grapple_nodeAB				
				grapple_neighbors[grapple_nodeA] = [grapple_nodeB]
				grapple_neighbors[grapple_nodeB] = [grapple_nodeA]	
				
				indexA, indexB = grapple_nodeA.grapple_index, grapple_nodeB.grapple_index
				for node in grapple_nodes:
					if (node.grapple_index == indexA) and (node.config.ee1_grappled== grapple_nodeA.config.ee1_grappled):
						grapple_neighbors[node].append(grapple_nodeA)
						grapple_neighbors[grapple_nodeA].append(node)
						
					if (node.grapple_index == indexB) and (node.config.ee1_grappled== grapple_nodeB.config.ee1_grappled):
						grapple_neighbors[node].append(grapple_nodeB)
						grapple_neighbors[grapple_nodeB].append(node)
								
				grapple_nodes.append(grapple_nodeA)
				grapple_nodes.append(grapple_nodeB)
			else:
				break
				
		grapple_paths = []
		grapple_container = [init_node]
		grapple_visited = {init_node: [init_node]}
		while len(grapple_container)>0:
			current_grapple = grapple_container.pop(0)
			if current_grapple.grapple_index == goal_node.grapple_index and  current_grapple.config.ee1_grappled == goal_node.config.ee1_grappled:
				grapple_found = True
				grapple_visited[goal_node] = grapple_visited[current_grapple] + [goal_node]
				grapple_paths = grapple_visited[goal_node]
				break				
			suc_grapples = grapple_neighbors[current_grapple]
			for suc in suc_grapples:
				if suc not in grapple_visited:
					grapple_container.append(suc)
					grapple_visited[suc] = grapple_visited[current_grapple] + [suc]

	for ee1_point in ee1_list:
		ee1_grapple_points.append(ee1_point)
	for ee2_point in ee2_list:
		ee2_grapple_points.append(ee2_point)
	return grapple_paths

def grapple_connect(start_grapple_index, end_grapple_index, spec, ee1_grapple_points, ee2_grapple_points):
	""" Generate a pair of nodes that connect 2 different grapple points, the pair has the same position 
		but the grapple points are different
	"""
	if (start_grapple_index in ee1_grapple_points) and (end_grapple_index in ee1_grapple_points):
		return None
	if (start_grapple_index in ee2_grapple_points) and (end_grapple_index in ee2_grapple_points):
		return None
	grapple_connect_found = False
	rand_count = 0
	index_reversed = False
	while (not grapple_connect_found) and (rand_count<100):
		rand_count += 1
		if (start_grapple_index in ee1_grapple_points) or (end_grapple_index in ee2_grapple_points):
			A_grapple_index, B_grapple_index = start_grapple_index, end_grapple_index
		elif (start_grapple_index in ee2_grapple_points) or (end_grapple_index in ee1_grapple_points):
			A_grapple_index, B_grapple_index = end_grapple_index, start_grapple_index
			index_reversed = True
		else:
			if random.random() < 0.5:
				A_grapple_index, B_grapple_index = start_grapple_index, end_grapple_index
			else:
				A_grapple_index, B_grapple_index = end_grapple_index, start_grapple_index
				index_reversed = True
		rand_config = config_random(spec, A_grapple_index, ee1_grapple_points, ee2_grapple_points, ee1_grappled=True, min_length=False)
		rand_node = GraphNode(spec, rand_config)	
		grapple_nodeAB = grapple_node_create(rand_node, spec.grapple_points[B_grapple_index])
		if grapple_nodeAB is not None:
			grapple_connect_found = True
			(grapple_nodeA, grapple_nodeB) = grapple_nodeAB
			grapple_nodeB.neighbors.append(grapple_nodeA)
			grapple_nodeA.neighbors.append(grapple_nodeB)	
			
		del rand_node
		if grapple_connect_found:
			if 	index_reversed:
				return (grapple_nodeB, grapple_nodeA)
			else:
				return (grapple_nodeA, grapple_nodeB)
	return None
					
def main(arglist):
	spec = ProblemSpec(arglist[0])
	fileout = arglist[1]
	num_segments = spec.num_segments
	num_grapples = len(spec.grapple_points)
		
	init_node = GraphNode(spec, spec.initial)
	goal_node = GraphNode(spec, spec.goal)
	start = time.time()
	
	# list of grapple points where only ee1 or ee2 attachs to
	ee1_grapple_points = []
	ee2_grapple_points = []
	Graph = {}
	
	colided_count = 0
	total_count = 0
	deleted_count = 0
	used_count = 0
	print("\nStart connecting the grapple points")
	# Generate a possible grapple changing path
	grapple_paths = find_grapple_path(init_node, goal_node, spec, ee1_grapple_points, ee2_grapple_points)
	print("\nFinish connecting the grapple point")
	print("\nGrapple paths include %d sub-paths" % (len(grapple_paths)-1))
	
	total_paths = []
	# Finding the sub_paths between each pair of consecutive grapple nodes 
	for i in range(len(grapple_paths)-1): #)
		OK = False
		paths = []

		start_node, end_node = grapple_paths[i], grapple_paths[i+1] 
		print("\n\nStart finding the sub-path no. %d from:" % (i+1))
		print(start_node.config)
		print("to:")
		print(end_node.config)
		
		if (start_node.grapple_index != end_node.grapple_index) :
			paths = [start_node, end_node]
			print("\nSub-path no. %d has been found" % (i+1))
		else:
			grapple_index = start_node.grapple_index
			init_nodes_list = [start_node]
			goal_nodes_list = [end_node]
			init_final_list = []
			goal_final_list = []
			
			prmlimit =  max(2,len(spec.grapple_points) + num_segments - 3)
			rand_count = 0
			init_goal_count = 0
			sub_start = time.time()
			
			while (not OK): #and (time.time() - start < 240)
				if (i < len(grapple_paths)-2):			
					if (init_goal_count >5*prmlimit) or (prmlimit==10 and rand_count>20):
						new_end_found = False
						while (not new_end_found):
							grapple_end_next = grapple_connect(grapple_index, grapple_paths[i+2].grapple_index, spec, ee1_grapple_points, ee2_grapple_points)
							if grapple_end_next is not None:
								new_end_found = True
								end_node, next_start_node = grapple_end_next	
								print("\n\tRestart finding the sub-path no. %d from:" % (i+1))
								print("\t", start_node.config)
								print("\tto:")
								print("\t",end_node.config)						
						grapple_paths[i+1], grapple_paths[i+2]  = end_node, next_start_node
						goal_nodes_list = [end_node]
						init_goal_count = 0
						rand_count = 0
				
				if rand_count > 20 and prmlimit < 10:
					prmlimit += 1
					rand_count = 0
					init_goal_count = 0
					#print("prmlimit = ", prmlimit)
										
				rand_config = config_random(spec, grapple_index, ee1_grapple_points, ee2_grapple_points, ee1_grappled=start_node.config.ee1_grappled, min_length=True)
				total_count += 1;
				rand_node = GraphNode(spec, rand_config)
				if (not test_collision(rand_node)) or (not test_environment_bounds(rand_node)):
					colided_count += 1
					del rand_node
				else:
					in_use = False
					init_in_use = False
					goal_in_use = False
					
					if len(init_nodes_list) < prmlimit:
						search_list = list(init_nodes_list)
					elif len(goal_nodes_list) < prmlimit:
						search_list = list(goal_nodes_list)
						init_goal_count += 1
					else:
						search_list = list(init_nodes_list + goal_nodes_list)
												
					init_goal_connect = []
					path_connect = {}
					for node in search_list:
						in_use = False
						if possible_path(node, rand_node):								
							path = nodes_path(node, rand_node)
							if (path is not None):		
								if (len(init_nodes_list) == prmlimit) and (len(goal_nodes_list) == prmlimit):
									if node in init_nodes_list:
										rand_count += 1
										#print("found 2")
										init_goal_connect.append(node)
										init_final_list = init_nodes_list + [rand_node]	
										path_connect[(node, rand_node)]	= path								
										init_in_use = True
										
									if node in goal_nodes_list:
										rand_count += 1
										#if init_in_use:
										#	print("found")	
										init_goal_connect.append(node)	
										goal_final_list = goal_nodes_list + [rand_node]
										path_connect[(node, rand_node)]	= path										
										goal_in_use = True										
																	
								if (len(init_nodes_list) < prmlimit) and (node in init_nodes_list) and (rand_node not in init_nodes_list):
									init_nodes_list.append(rand_node)
									in_use = True								
								if (len(goal_nodes_list) < prmlimit) and (node in goal_nodes_list) and (rand_node not in goal_nodes_list):	
									goal_nodes_list.append(rand_node)
									in_use = True
								
								if in_use:
									used_count += 1
									node.neighbors.append(rand_node)
									rand_node.neighbors.append(node)
									Graph[(node, rand_node)] = path
									Graph[(rand_node, node)] = list(p for p in reversed(path))
										
					if init_in_use and goal_in_use:
						used_count += 1
						for connect_node in init_goal_connect:
							connect_node.neighbors.append(rand_node)
							rand_node.neighbors.append(connect_node)
							Graph[(connect_node, rand_node)] = path_connect[(connect_node, rand_node)]
							Graph[(rand_node, connect_node)] = list(p for p in reversed(path_connect[(connect_node, rand_node)]))			
					elif not in_use:
						deleted_count += 1
						del rand_node
					
					#print("init length = %d" % len(init_nodes_list))	
					#print("goal length = %d" % len(goal_nodes_list))	
					
					# Check if there is a node connect to both initial and goal node
					for node in init_final_list:
						if node in goal_final_list:
							#print("OK1")
							OK = True
							break
			
			#Start searching for sub paths (bread-first-search)
			search_start = time.time()
			init_container = [start_node]
			init_visited = {start_node: [start_node]}
			while len(init_container) > 0:
				current_node = init_container.pop(0)
				
				if (current_node, end_node) in Graph:
					print("\nSub-path no. %d has been found" % (i+1))
					init_visited[end_node] = init_visited[current_node] + [end_node]
					main_path = init_visited[end_node]
					for i in range(len(main_path)-1):
						paths = paths + Graph[(main_path[i],main_path[i+1])]
					break
					
				successors = current_node.get_successors()
				for suc in successors:
					if suc not in init_visited:
						init_container.append(suc)
						init_visited[suc] = init_visited[current_node] + [suc]

		total_paths = total_paths + paths 
		
	end = time.time()
	print("\n\n***The calculation has finished. The performance summary is following:\n")
	print("\tTest set: %s" % arglist[0][-10:])
	print("\tNumber of random nodes created: %d" % total_count)
	print("\tNumber of colided nodes: %d" % colided_count)
	print("\tNumber of random nodes deleted: %d" % deleted_count)
	print("\tNumber of random nodes in use: %d" % used_count)
	print("\tNumber of possible path in the Graph: %d" % len(Graph))
	print("\tElapsed time: %f \n" % (end - start))
	print("**********\n")
	
	with open(fileout, "w") as f:
		for node in total_paths:
			f.write(str(node.config) + "\n")
	f.close()	
		
if __name__== '__main__':
	main(sys.argv[1:])