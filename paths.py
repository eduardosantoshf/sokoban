from consts import Tiles, TILES
from tree_search import *

#------------------------------------------------------------------------------------------

def breathSearch(start, end, map):
	#map size
	horizontal_tiles, vertical_tiles = map.size

	#visited array
	visited_position = [[0] * horizontal_tiles for x in range(vertical_tiles)]

	#queue
	search_queue = []

	#enqueue data
	def enqueue(data):
		search_queue.append(data)
	
	#dequeue data
	def dequeue():
		x, y, path, man = search_queue.pop(0)
		return x, y, path, man

	#recursive call
	def recursive():
		#while is not empty
		while search_queue:
			#pop item
			x, y, path, man = dequeue()

			#sort by heuristic
			search_queue.sort(key = lambda x: x[3])

			#if is already visited or box,box_on_goal or wall
			if visited_position[y][x] or (map.get_tile((x,y)) == Tiles.BOX) or (map.get_tile((x,y)) == Tiles.BOX_ON_GOAL) or (map.get_tile((x,y)) == Tiles.WALL): 
				continue 
			
			#set position as visited
			visited_position[y][x] = 1

			#if it reaches the end
			if (x, y) == end: return path

			#recursive call in all directions
			if 0 <= (y - 1) < vertical_tiles:
				enqueue((x, y - 1, path + "w", manhattan((x, y-1), end)))
			if 0 <= (y + 1) < vertical_tiles:
				enqueue((x, y + 1, path + "s",  manhattan((x, y+1), end)))
			if 0 <= (x - 1) < horizontal_tiles:
				enqueue((x - 1, y, path + "a",  manhattan((x-1, y), end)))
			if 0 <= (x + 1) < horizontal_tiles:
				enqueue((x + 1, y, path + "d",  manhattan((x+1, y), end)))

		#if there is no solution
		return None

	#enqueue first one
	enqueue((start[0], start[1], "", 0))

	#start recursive
	return recursive()

#------------------------------------------------------------------------------------------
	
def depthSearch(start, end, map):
	#map size
	horizontal_tiles, vertical_tiles = map.size

	#visited array
	visited_position = [[0] * horizontal_tiles for x in range(vertical_tiles)]

	# recursive call
	def recursive(pos, path):
		x, y = pos

		#if is already visited or box, box_on_goal or wall
		if visited_position[y][x] or (map.get_tile((x,y)) == Tiles.BOX) or (map.get_tile((x,y)) == Tiles.BOX_ON_GOAL) or (map.get_tile((x,y)) == Tiles.WALL):
			return None 

		#set position as visited
		visited_position[y][x] = 1

		#if reaches the end
		if pos == end: return path   

		#recursive call in all directions
		if 0 <= (y - 1) < vertical_tiles:
			path_2_goal = recursive((x, y - 1), path + "w")

			if path_2_goal != None: return path_2_goal

		if 0 <= (x - 1) < horizontal_tiles:
			path_2_goal = recursive((x - 1, y), path + "a")

			if path_2_goal != None: return path_2_goal

		if 0 <= (y + 1) < vertical_tiles:
			path_2_goal = recursive((x, y + 1), path + "s")

			if path_2_goal != None: return path_2_goal

		if 0 <= (x + 1) < horizontal_tiles:
			path_2_goal = recursive((x + 1, y), path + "d")

			if path_2_goal != None: return path_2_goal		
		
		#if there is no solution
		return None

	#start recursive
	return recursive(start, "")

#------------------------------------------------------------------------------------------

# manhattan distance
def manhattan(initial, goal):
	return abs(initial[0] - goal[0]) + abs(initial[1] - goal[1])

