from abc import ABC, abstractmethod
from consts import TILES
from copy import deepcopy
from paths import *
import asyncio
import time

##################################################################################################

class SearchNode:
    def __init__(self, cur_pos, mapa, cost, boxes, parent = None, directions = (0,0), key = None):
        self.cur_pos = cur_pos
        self.parent = parent
        self.directions = directions
        self.key = key
        self.cost = cost
        self.boxes = boxes
        # caso não seja o root
        if parent is not None:
            #get keeper path to box
            self.path = breathSearch(mapa.keeper, (cur_pos[0] - directions[0], cur_pos[1] - directions[1]), mapa)
            # if it is possible
            if self.path is not None:
                # add the push to path
                self.path += self.key
            # move
            self.map = self.makeMove(mapa)
        # caso seja o root
        else:
            self.path = ""
            self.cost = 0
            self.map = mapa

    # toString
    def __str__(self):
        return  "(" + str(int(self.cost)) + ")"

    #------------------------------------------------------------------------------------------

    # move
    def makeMove(self, mapa):
        box_pos = self.cur_pos[0] + self.directions[0], self.cur_pos[1] + self.directions[1]
        # remove keeper and box
        mapa.clear_tile(mapa.keeper)
        mapa.clear_tile(self.cur_pos)
        # set keeper and box on new positions
        mapa.set_tile(box_pos, Tiles.BOX)
        mapa.set_tile(self.cur_pos, Tiles.MAN)
        return mapa    

##################################################################################################

class SearchTree:
    def __init__(self, mapa, strategy = 'a_star'):
        self.strategy = strategy
        # starting the tree
        self.root = SearchNode(mapa.keeper, mapa, 0, [])
        self.open_nodes = [self.root]
        self.goals = mapa.filter_tiles([Tiles.GOAL, Tiles.MAN_ON_GOAL, Tiles.BOX_ON_GOAL])
        # simple deadlocks
        self.first_type_deadlocks = self.firstTypeDeadlocks(mapa)
        # backtracking
        self.visited_nodes = {}
        # possible directions
        self.possible_keys = [(-1, 0, "a"), (0, 1, "s"), (1, 0, "d"), (0, -1, "w")]
        # precalculated distances
        self.distanceToGoal = self.precalculate_distances(mapa)
    
    #------------------------------------------------------------------------------------------

    # path to node
    def get_path(self, node):
        if node.parent == None:
            return node.path
        path = self.get_path(node.parent)
        path += node.path
        return path

    #------------------------------------------------------------------------------------------

    def precalculate_distances(self, mapa):
        
        queue = []
        distanceToGoal = {}

        #initializing
        horizontal_tiles, vertical_tiles = mapa.size
        matrix = [[5000] * vertical_tiles for x in range(horizontal_tiles)]

        for g in self.goals: distanceToGoal[g] = matrix
        
        for goal in self.goals:
            distanceToGoal[goal][goal[0]][goal[1]] = 0
            queue.append(goal)

            while queue:
                position = queue.pop(0)

                for d in [(-1, 0), (0, 1), (1, 0), (0, -1)]:
                    # next box position
                    box_pos = (position[0] + d[0], position[1] + d[1])
                    keeper_dir = (2 * d[0], 2 * d[1])
                    # next keeper position
                    keeper = (position[0] + keeper_dir[0], position[1] + keeper_dir[1])
                    
                    if distanceToGoal[goal][box_pos[0]][box_pos[1]] == 5000:
                        # check if we can pull the box
                        if not (mapa.get_tile(box_pos) == Tiles.WALL) and not (mapa.get_tile(keeper) == Tiles.WALL):
                            distanceToGoal[goal][box_pos[0]][box_pos[1]] = distanceToGoal[goal][position[0]][position[1]] + 1
                            queue.append(box_pos)
        
        return distanceToGoal

    #------------------------------------------------------------------------------------------

    # search for a solution
    async def search(self):
        while len(self.open_nodes) > 0:
            await asyncio.sleep(0)
            current_state = self.open_nodes.pop(0)

            if (current_state.map.completed):
                return self.get_path(current_state)

            new_nodes = self.getMoves(current_state.map, current_state)
            self.add_to_open(new_nodes)
        return None

    #------------------------------------------------------------------------------------------

    # possible nodes to expand
    def getMoves(self, mapa, node):
        #nodes to expand
        moves = []
        # boxes
        for box_x, box_y in mapa.boxes:
            # moves
            for dx, dy, key in self.possible_keys:
                # next move
                move = (box_x + dx, box_y + dy)
                # tile
                tl = mapa.get_tile((move[0],move[1]))
                # copy the map so we dont change the original one
                mapCopy = deepcopy(mapa)
                # boxes after move
                new_boxes = [b for b in mapa.boxes if b != (box_x, box_y)]
                new_boxes.append(move)
                # new node
                new_node = SearchNode((box_x, box_y),mapCopy, self.heuristic(new_boxes), new_boxes, node,(dx,dy),key)
                #conditions to add node
                if (
                    # check if it is a valid position to move
                    not (tl == Tiles.BOX)
                    and not (tl == Tiles.BOX_ON_GOAL)
                    and not (tl == Tiles.WALL)
                    # check if it is possible
                    and new_node.path is not None
                    # check basic deadlocks
                    and (move) not in self.first_type_deadlocks
                    # check freeze deadlocks
                    and self.freeze_deadlock(mapCopy, (box_x, box_y, dx, dy))):
                    
                    # if state was never visited
                    if (str(new_node.boxes) not in self.visited_nodes):
                        moves.append(new_node)
                        self.visited_nodes[str(new_node.boxes)] = new_node.cur_pos
                    # if state was visited, but keeper cannot reach it
                    elif (depthSearch(new_node.cur_pos, self.visited_nodes.get(str(new_node.boxes)), mapCopy) == None):
                        moves.append(new_node)
                        self.visited_nodes[str(new_node.boxes)] = new_node.cur_pos

        return moves

    #------------------------------------------------------------------------------------------

    # manhattan distance
    def manhattan(self, initial, goal):
        return abs(initial[0] - goal[0]) + abs(initial[1] - goal[1])

    #------------------------------------------------------------------------------------------

    # greedy heuristic
    def heuristic(self, boxes):
        costs = []

        # initializing
        for box in boxes:
            for goal in self.goals: costs.append((box, goal))
        
        # array of costs sorted by distance to goal using precalculated distances
        costs =  sorted(costs, key = lambda p: self.distanceToGoal[p[1]] [p[0][0]] [p[0][1]], reverse=True)

        m_boxes = []
        m_goals = []
        h = 0

        while len(costs) > 0:
            (b, goal) = costs.pop(0)

            if b not in m_boxes and goal not in m_goals:
                h += self.manhattan(b, goal)
                m_boxes.append(b)
                m_goals.append(goal)
        
        for box in boxes:
            if b not in m_boxes: 
                h += min([self.manhattan(goal, box) for goal in self.goals])
        
        return h

    #------------------------------------------------------------------------------------------

    # simple deadlocks
    def firstTypeDeadlocks(self, map):
        #size of the map
        horizontalTiles, verticalTiles = map.size
        hor_range = range(horizontalTiles)
        ver_range = range(verticalTiles)

        #visited array
        visited = [[0] * verticalTiles for x in hor_range]

        #deadlocks
        dead = set()

        copy = deepcopy(map)

        # replace boxes with floors and goals with boxes
        for x in hor_range:
            for y in ver_range:
                if(copy.get_tile((x,y)) == Tiles.BOX):
                    copy.clear_tile((x,y))
                    copy.set_tile((x,y), Tiles.FLOOR)
                if(copy.get_tile((x,y)) == Tiles.GOAL):
                    copy.clear_tile((x,y))
                    copy.set_tile((x,y), Tiles.BOX)

        #recursive call
        def recursive(pos):
            x, y = pos

            ## already visited or hits a wall
            if visited[x][y] or copy.get_tile(pos) == Tiles.WALL:
                return

            #set position to visited
            visited[x][y] = 1

            #recursive calls
            if x in hor_range and y + 2 in ver_range and not (copy.get_tile((x, y + 2)) == Tiles.WALL ):
                recursive((x, y + 1))

            if x + 2 in hor_range and y in ver_range and not (copy.get_tile((x + 2, y)) == Tiles.WALL ):
                recursive((x + 1, y))

            if x in hor_range and y - 2 in ver_range and not (copy.get_tile((x, y - 2)) == Tiles.WALL ):
                recursive((x, y - 1))

            if x - 2 in hor_range and y in ver_range and not (copy.get_tile((x - 2, y)) == Tiles.WALL ):
                recursive((x - 1, y))
                
            return

        # call recursive if is a box on goal so we can pull it
        [recursive((x,y)) for x in hor_range for y in ver_range if(copy.get_tile((x,y)) == Tiles.BOX_ON_GOAL)]
        
        # adding dead positions
        [dead.add((x,y)) for x in hor_range for y in ver_range if not visited[x][y]]

        return dead

    #------------------------------------------------------------------------------------------

    def freeze_deadlock(self,mapa, positions, visitedBoxes = None):
        #box positions
        boxes = (positions[0], positions[1])

        #keys
        keys = (positions[2], positions[3])

        #next moves
        next_x = boxes[0] + keys[0]
        next_y = boxes[1] + keys[1]
    
        #not a deadlock
        if mapa.get_tile((next_x, next_y)) == Tiles.BOX_ON_GOAL:
            return True

        if not visitedBoxes:
            npos = boxes[0] + keys[0], boxes[1] + keys[1]

            #clear tiles to move
            mapa.clear_tile(mapa.keeper)
            mapa.clear_tile((boxes[0], boxes[1]))

            #move tiles
            mapa.set_tile((boxes[0], boxes[1]), Tiles.MAN)
            mapa.set_tile(npos, Tiles.BOX)
            visitedBoxes = set()

        #add coordinates of box
        visitedBoxes.add((next_x, next_y))
        flag = False

        # Verifing for vertical axis
        for dir_x, dir_y in [(0, -1), (0, 1)]:
            #next move
            next_move = (next_x + dir_x, next_y + dir_y)

            #if is wall
            if mapa.get_tile(next_move) == Tiles.WALL:
                flag = False
                break

            #if is box on goal
            if mapa.get_tile(next_move) == Tiles.BOX_ON_GOAL:
                #recursive call
                if next_move not in visitedBoxes:
                    flag = self.freeze_deadlock(mapa, (next_x, next_y, dir_x, dir_y), visitedBoxes)
            
            #if is already a basic deadlock
            if next_move in self.first_type_deadlocks:
                continue

            #else is not a deadlock
            else:
                return True

        # Verifing for horizontal axis
        for dir_x, dir_y in [(-1, 0), (1, 0)]:
            #next move
            next_move = (next_x + dir_x, next_y + dir_y)

            #if is wall
            if mapa.get_tile(next_move) == Tiles.WALL:
                flag = False
                break

            #if is box on goal
            if mapa.get_tile(next_move) == Tiles.BOX_ON_GOAL:
                #recursive call
                if next_move not in visitedBoxes:
                    flag = self.freeze_deadlock(mapa, (next_x, next_y, dir_x, dir_y), visitedBoxes)
            
            #if is already a basic deadlock
            if next_move in self.first_type_deadlocks:
                continue

            #else is not a deadlock
            else:
                return True

        return flag

    #------------------------------------------------------------------------------------------

    def add_to_open(self,new_nodes):
        if self.strategy == 'a_star':
            self.open_nodes.extend(new_nodes)
            self.open_nodes.sort(key=lambda x: x.cost)