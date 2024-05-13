"""  
*	Author: Duy Long Tran
*  	Implementing uniform cost search and A* serach algorithms to solve the Sokoban puzzle
*
*  	basic compile eg. with:
*  	python solver.py input.txt output.txt
*  
************************************
"""


import sys
import numpy as np
from sokoban_map import SokobanMap
import time
OBS_SYM = '#'
DEAD_COUNT = 0
		
class PriorityQueue:
	"""A class to control the queue of the search algorithm
		function: 
			put(): add an element to the object
			get(): take the element with smallest cost value
	"""
	def __init__(self):
		self.elements = []

	def empty(self):
		return len(self.elements) == 0

	def put(self, priority, item):
		self.elements.append((priority, item))

	def get(self):
		index = np.argmin([anelement[0] for anelement in self.elements])
		next = self.elements[index]
		self.elements.remove(next)
		return next[1]
		
def main(argList):
	global DEAD_COUNT
	map_file = argList[0]
	game_map = SokobanMap(map_file)
	fileout = argList[1]
	
	#Reading the game map
	boxes = game_map.box_positions
	tgts = game_map.tgt_positions
	player = game_map.player_position
	obstacles = game_map.obstacle_map
	start_map = boxes
	start_map.sort()
	start_map.append(player)
	start_map = tuple(start_map)
	print("\nThe game map has been read. Now calculating for a solution...\n")
	start = time.time()
	(path, explored_len, frontier_max) = a_star_search(start_map, tgts, obstacles)
	end = time.time()
	
	#Write down the solution to the output file
	if (path):
		solution = ""
		with open(fileout, "w") as f:
			for i in range(len(path)):
				f.write(path[i])
				solution += path[i]
				if (i!=len(path)-1): 
					f.write(',')
					solution += ","
		f.close()
		
		print("The calculation is finished. The results are following: \n")
		print("\tNumber of explored states: %d" % explored_len)
		print("\tMaximum size of frontier: %d" % frontier_max)
		print("\tNumber of deadlocks: %d" % DEAD_COUNT)
		print("\tElapsed time: %f" % (end-start))
		print("\tNumber of steps: %d" % len(path))
		print("\tSolution: " + solution + "\n")
		
def uniform_search(start_map, tgts, obstacles):
	"""Uniform cost search implementation
		input: 
			start_map: starting state, 
			tgts: list of target positions, 
			obstacles: list of obstacles positions
		output: 
			path: list of step in the solution
			explored_len: number of explored nodes
			frotier_max: maximum size of the frontier
	"""
	frontier = PriorityQueue()
	frontier.put(0, start_map)
	path = None
	cost_so_far = {}
	came_from = {}
	came_from[start_map] = None
	cost_so_far[start_map] = 0
	frontier_max = 0
	while not frontier.empty():
		if frontier_max < len(frontier.elements): 
			frontier_max = len(frontier.elements)
		current = frontier.get()
		
		if finish(current, tgts):
			path = []
			while came_from[current]:
				path.append(move(came_from[current][-1], current[-1]))
				current = came_from[current]
			path = list(p for p in reversed(path))	
			break
		
		for next in map_neighbors(current, obstacles):
			new_cost = cost_so_far[current] + 1
			
			if finish(next, tgts):
				cost_so_far[next] = 0
				came_from[next] = current
				frontier.put(0, next)
				break
			elif not deadlocks(next, tgts, obstacles):
				if next not in cost_so_far or new_cost < cost_so_far[next]:
					cost_so_far[next] = new_cost
					came_from[next] = current
					frontier.put(new_cost, next)
	return path, len(cost_so_far), frontier_max		
	
def a_star_search(start_map, tgts, obstacles):
	"""A* search implementation
		input: 
			start_map: starting state, 
			tgts: list of target positions, 
			obstacles: list of obstacles positions
		output: 
			path: list of step in the solution
			explored_len: number of explored nodes
			frotier_max: maximum size of the frontier
	"""
	frontier = PriorityQueue()
	frontier.put(0, start_map)
	path = None
	cost_so_far = {}
	came_from = {}
	came_from[start_map] = None
	cost_so_far[start_map] = 0
	frontier_max = 0
	while not frontier.empty():
		if frontier_max < len(frontier.elements): 
			frontier_max = len(frontier.elements)
		current = frontier.get()
		
		if finish(current, tgts):
			path = []
			while came_from[current]:
				path.append(move(came_from[current][-1], current[-1]))
				current = came_from[current]
			path = list(p for p in reversed(path))	
			break
		
		for next in map_neighbors(current, obstacles):
			new_cost = cost_so_far[current] + 1
			
			if finish(next, tgts):
				cost_so_far[next] = 0
				came_from[next] = current
				frontier.put(0, next)
				break
			elif not deadlocks(next, tgts, obstacles):
				if next not in cost_so_far or new_cost < cost_so_far[next]:
					cost_so_far[next] = new_cost
					came_from[next] = current
					frontier.put(new_cost+heuristic(next, tgts), next) #
	return path, len(cost_so_far), frontier_max
	
def map_neighbors(state, obstacles):
	"""Finding the next possible state to move:
		output:
			neighbors: list of next states
	"""
	player = state[-1]
	boxes = state[0:-1]
	neighbors = []
	for r in range(player[0]-1,player[0]+2):
		for c in range(player[1]-1,player[1]+2):
			if cb_dist((r,c),player)==1 and obstacles[r][c] != OBS_SYM:
				new_y = r
				new_x = c
				new_box_y = 2*r-player[0]
				new_box_x = 2*c-player[1]
				if (r,c) in boxes:
					if obstacles[new_box_y][new_box_x] != OBS_SYM and ((new_box_y,new_box_x) not in boxes):
						new_neighbor = list(b for b in boxes)
						new_neighbor.remove((r,c))
						new_neighbor.append((new_box_y, new_box_x))
						new_neighbor.sort()
						new_neighbor.append((new_y, new_x))
						neighbors.append(tuple(new_neighbor))
				else:
					new_neighbor = list(b for b in boxes)
					new_neighbor.append((new_y, new_x))
					neighbors.append(tuple(new_neighbor))
	return neighbors

def heuristic(state, tgts):
	"""Heuristic values for A* search algorithm:
		output:
			bt_dist: hamming distance between boxes and targets
	"""
	player = state[-1]
	bt_dist = 0
	for box in state[0:-1]:
		if box in tgts:
			bt_dist += 1
	
	return bt_dist

def move(stateA, stateB):
	"""Finding a move to change from stateA to stateB
		output: 'l' or 'r' or 'u' or 'd'
	"""
	if stateA[0]>stateB[0]:
		return 'u'
	if stateA[0]<stateB[0]:
		return 'd'
	if stateA[1]>stateB[1]:
		return 'l'
	if stateA[1]<stateB[1]:
		return 'r'
	
def finish(state, tgts):
	"""Checking if the state is the goal
		output: True if state is the goal, False otherwise
	"""
	for box in state[0:-1]:
		if box not in tgts:
			return False
	return True	

def deadlocks(state, tgts, obstacles):
	"""Checking if the state is a deadlock
		output: True if the state is a deadlock, False otherwise
	"""
	global DEAD_COUNT
	boxes = state[0:-1]
	for box in boxes:
		if box not in tgts:
			if obs_deadlock(box, obstacles):
				DEAD_COUNT += 1
				return True
	return False

def obs_deadlock(box, obstacles):
	"""Checking if a box can be moved
		output: True if the box cannot be moved, False otherwise
	"""
	box_x = box[0]
	box_y = box[1]
	if (obstacles[box_x][box_y-1]==OBS_SYM or obstacles[box_x][box_y+1]==OBS_SYM) and (obstacles[box_x-1][box_y]==OBS_SYM or obstacles[box_x+1][box_y]==OBS_SYM):
		return True
	return False
	
def cb_dist(objA, objB):
	"""Calculating the city-block distance between objectA and objectB
	"""
	return (abs(objA[0]-objB[0]) + abs(objA[1]-objB[1]))	
	
if __name__ == '__main__':
    main(sys.argv[1:])