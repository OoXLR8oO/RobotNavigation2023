from collections import deque
from Cell import *

from heapq import heappop, heappush

import math
from queue import PriorityQueue
import sys
import random

class Agent:
    def __init__(self, maze, algorithm):
        self.maze = maze
        self.path = []
        self.visited = set()
        self.move_direction = []
        self.depth_limit = 0
        self.algorithm = algorithm
        self.goal_found = False
        self.nodes = [self.maze.get_cell(*self.maze.start)]
        
    # Get all valid (not visited and not an obstacle) neighbours of a given Cell
    def get_neighbours(self, cell):
        neighbours = []
        row = cell.x
        col = cell.y
        if row - 1 >= 0: 
            if self.maze.get_cell(row - 1, col).type != 1: # CHECK UP
                neighbours.append(self.maze.get_cell(row - 1, col))
                if self.maze.get_cell(row - 1, col) is not self.maze.get_cell(*self.maze.start) and self.maze.get_cell(row - 1, col) not in self.maze.goals: # (1)
                    self.maze.get_cell(row - 1, col).type = 3
                    if self.maze.get_cell(row - 1, col) not in self.nodes:
                        self.nodes.append(self.maze.get_cell(row - 1, col))
        if col - 1 >= 0:
            if self.maze.get_cell(row, col - 1).type != 1: # CHECK LEFT
                neighbours.append(self.maze.get_cell(row, col - 1))
                if self.maze.get_cell(row, col - 1) is not self.maze.get_cell(*self.maze.start) and self.maze.get_cell(row, col - 1) not in self.maze.goals: # (1)
                    self.maze.get_cell(row, col - 1).type = 3
                    if self.maze.get_cell(row, col - 1) not in self.nodes:
                        self.nodes.append(self.maze.get_cell(row, col - 1))
        if row + 1 < self.maze.height:
            if self.maze.get_cell(row + 1, col).type != 1: # CHECK DOWN
                neighbours.append(self.maze.get_cell(row + 1, col))
                if self.maze.get_cell(row + 1, col) is not self.maze.get_cell(*self.maze.start) and self.maze.get_cell(row + 1, col) not in self.maze.goals: # (1)
                    self.maze.get_cell(row + 1, col).type = 3
                    if self.maze.get_cell(row + 1, col) not in self.nodes:
                        self.nodes.append(self.maze.get_cell(row + 1, col))
        if col + 1 < self.maze.width:
            if self.maze.get_cell(row, col + 1).type != 1: # CHECK RIGHT
                neighbours.append(self.maze.get_cell(row, col + 1))
                if self.maze.get_cell(row, col + 1) is not self.maze.get_cell(*self.maze.start) and self.maze.get_cell(row, col + 1) not in self.maze.goals: # (1)
                    self.maze.get_cell(row, col + 1).type = 3
                    if self.maze.get_cell(row, col + 1) not in self.nodes:
                        self.nodes.append(self.maze.get_cell(row, col + 1))
        return neighbours
        # (1): If Neighbour is not of type 1, 2 or 7, change type to 3 ('Expanded' type)
    
    # Converts Path of Cell objects to Path of coordinates
    def get_path_in_coords(self):
        path_coords = []
        for cell in self.path:
            path_coords.insert(0, [cell.x, cell.y])
        return path_coords

    # Change Cells along Path to type to 4 ('Path' type)
    def draw_path(self):
        if self.algorithm == "dfs": # Band-Aid fix as recursion doesn't add start node at the end.
            self.path.append(self.maze.get_cell(*self.maze.start))
        for cell in self.path:
            if cell not in self.maze.goals and cell is not self.maze.get_cell(*self.maze.start):
                cell.type = 4

    # Translate given Path into directions
    def get_directions(self):
        for i in range(len(self.path) - 1):
            # Grab two adjacent values in Path
            curr = self.path[i]
            next = self.path[i+1]
            # Get difference in x and y values for current and new Cells
            row_diff = next.x - curr.x
            col_diff = next.y - curr.y
            # Compute a direction based on above results
            if row_diff == 1:
                self.move_direction.append("up;")
            elif row_diff == -1:
                self.move_direction.append("down;")
            elif col_diff == 1:
                self.move_direction.append("left;")
            elif col_diff == -1:
                self.move_direction.append("right;")
        return list(reversed(self.move_direction))

    #=================================#
    #           ALGORITHMS            #
    #=================================#

    # BFS SEARCH
    def bfs(self):
        start = self.maze.get_cell(*self.maze.start) # '1', '0'

        queue = deque()
        queue.append(start)

        came_from = {}
        came_from[start] = None
        self.nodes.append(start)

        visited = set() # set to keep track of visited nodes
        
        while queue:
            current = queue.popleft()
            if current in self.maze.goals: # Terminating Condition/Goal State
                #print("BFS GOAL FOUND:", (current.x, current.y), "Type:", current.type)
                self.goal_found = True # Becomes True if Goal State is achieved
                break
            for neighbour in self.get_neighbours(current):
                if neighbour not in visited: # check if neighbour has not been visited before
                    queue.append(neighbour)
                    came_from[neighbour] = current
                    visited.add(neighbour) # mark neighbour as visited

        cell = current
        while cell != start:
            self.path.append(cell)
            cell = came_from[cell]
        self.path.append(start)


    # RECURSIVE DFS SEARCH
    def dfs_recursive(self, current):        
        if current in self.maze.goals: # Terminating Condition/Goal State
            self.goal_found = True
            return [current]       
        
        self.visited.add(current)
        for neighbour in self.get_neighbours(current):
            if neighbour not in self.visited:
                self.path = self.dfs_recursive(neighbour)
                if self.path:
                    return self.path + [current]
    
        return []

                
    # GBFS SEARCH            
    def gbfs(self):
        start = self.maze.get_cell(*self.maze.start) # '1', '0'
        
        frontier = deque()
        frontier.append((self.h_manhattan(start), start))
        
        came_from = {}
        came_from[start] = None
        
        while frontier:
            if len(frontier) > 1:
                frontier = sorted(frontier , key=lambda md: md[0])
            current = frontier[0]
            del frontier[0]
            current_cell = current[1]
            if current_cell in self.maze.goals:
                self.goal_found = True
                while current_cell is not start:
                    self.path.append(current_cell)
                    current_cell = came_from[current_cell]
                self.path.append(start)
                return self.path
            
            for neighbour in self.get_neighbours(current[1]):
                if neighbour not in came_from:
                    came_from[neighbour] = current_cell
                    frontier.append((self.h_manhattan(neighbour), neighbour))

        # If the frontier is empty and no solution is found, return an empty path and move_direction
        return self.path 
    
    # A* SEARCH
    def asa(self, start):        
        frontier = []
        frontier.append((self.h_manhattan(start), start))
            
        came_from = {}
        came_from[start] = None
        self.nodes.append(start)

        # Create a dictionary to keep track of g values (distance from start to cell)
        g_values = {start: 0}
        
        while frontier:
            frontier.sort(key=lambda i: i[0])
            current = frontier.pop(0) # (f_value, Cell)
            current_cell = current[1] # Cell
            
            if current_cell in self.maze.goals: # Terminating Condition/Goal State
                self.goal_found = True
                while current_cell is not start:

                    self.path.append(current_cell)
                    current_cell = came_from[current_cell]
                self.path.append(start)
                return self.path
            
            for neighbour in self.get_neighbours(current_cell):
                if neighbour not in came_from:
                    g_neighbour = g_values[current_cell] + 1
                    if neighbour in came_from and g_neighbour >= g_values.get(neighbour, float('inf')):
                            continue
                    # Update the g value and parent of the neighbour
                    g_values[neighbour] = g_neighbour
                    came_from[neighbour] = current_cell
                    
                    # Calculate the h value (Manhattan distance to closest goal)
                    h_value = self.h_manhattan(neighbour)
                    f_value = g_neighbour + h_value
                    frontier.append((f_value, neighbour))
        return self.path

    # DEPTH-LIMITED DFS SEARCH
    def dls_search(self):
        start = self.maze.get_cell(*self.maze.start)
        self.nodes.append(start)
        self.visited.add(start)
        
        self.depth_limit = 14

        self.path = self.dls_recursive(start, 0)
        if self.goal_found:
            return self.path
        else:
            return self.path

    def dls_recursive(self, current, depth):
        if current in self.maze.goals:
            self.goal_found = True
            return [current]

        if depth == self.depth_limit:
            return self.path

        for neighbour in self.get_neighbours(current):
            if neighbour not in self.visited:
                self.visited.add(neighbour)

                self.path = self.dls_recursive(neighbour, depth + 1)
                if self.goal_found:
                    return self.path + [current]
        return self.path
    
    # BFS ALL SEARCH
    def bfs2(self):
        start = self.maze.get_cell(*self.maze.start) # '1', '0'

        queue = deque()
        queue.append(start)

        came_from = {}
        came_from[start] = None
        self.nodes.append(start)

        visited = set() # set to keep track of visited nodes
        goals_to_visit = set(self.maze.goals) # set to keep track of unvisited goal nodes

        while queue and goals_to_visit:
            current = queue.popleft()
            if current in goals_to_visit:
                goals_to_visit.remove(current) # mark the goal as visited
            for neighbour in self.get_neighbours(current):
                if neighbour not in visited: # check if neighbour has not been visited before
                    queue.append(neighbour)
                    came_from[neighbour] = current
                    visited.add(neighbour) # mark neighbour as visited
            if len(goals_to_visit) == 0:
                self.goal_found = True

        # Generate the path
        goal_path = []
        cell = current
        while cell != start:
            goal_path.append(cell)
            cell = came_from[cell]
        goal_path.append(start)

        # Create the final path going through all goal cells to the start
        final_path = []
        for goal in self.maze.goals:
            if goal != start and goal in came_from:
                # Generate the path from the current goal to the start
                path_to_start = []
                cell = goal
                while cell != start:
                    path_to_start.append(cell)
                    cell = came_from[cell]
                path_to_start.append(start)
                # Add the path to the final path
                final_path += path_to_start

        # Add the path from the start to the first goal
        final_path += goal_path

        # Set the final path
        self.path = final_path
    
    # UNIFORM COST SEARCH    
    def ucs(self):
        start = self.maze.get_cell(*self.maze.start)
        frontier = [(0, start)]
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while frontier:
            current = min(frontier, key=lambda x: x[0])[1]
            frontier.remove((cost_so_far[current], current))

            if current in self.maze.goals:
                self.goal_found = True
                break

            for neighbour in self.get_neighbours(current):
                new_cost = cost_so_far[current] + 1  # assume uniform cost of 1
                if neighbour not in cost_so_far or new_cost < cost_so_far[neighbour]:
                    cost_so_far[neighbour] = new_cost
                    priority = new_cost
                    frontier.append((priority, neighbour))
                    came_from[neighbour] = current

        if self.goal_found:
            cell = current
            while cell != start:
                self.path.append(cell)
                cell = came_from[cell]
            self.path.append(start)
            self.path.reverse()
                            
                    
    def h_euclidean(self, cell):
        # Calculate the Euclidean distance between the current Cell position (x, y) and the goal position
        dist1 = math.sqrt((cell.x - self.maze.goal1[0]) ** 2 + (cell.y - self.maze.goal1[1]) ** 2)
        dist2 = math.sqrt((cell.x - self.maze.goal2[0]) ** 2 + (cell.y - self.maze.goal2[1]) ** 2)
        return min(dist1, dist2)

    def h_manhattan(self, cell):
        # Calculate the Manhattan distance between the current Cell position (x, y) and the closest goal position
        distances = [abs(cell.x - goal.x) + abs(cell.y - goal.y) for goal in self.maze.goals]
        return min(distances)
    
    # def h_manhattan_all(self, start_cell, end_cell):
    #     return abs(start_cell.x - end_cell.x) + abs(start_cell.y - end_cell.y)
    
    def h_manhattan_all(self, cell, goals):
        # Returns sum of distances from cell to all goals
        return sum(abs(cell.x - goal.x) + abs(cell.y - goal.y) for goal in goals)       

    # Runs a search strategy based on given user input
    def find_path(self):
        if self.algorithm == "bfs":
            self.bfs()
        elif self.algorithm == "dfs":
            self.dfs_recursive(self.maze.get_cell(*self.maze.start))
        elif self.algorithm == "gbfs":
            self.gbfs()
        elif self.algorithm == "as":
            self.asa(self.maze.get_cell(*self.maze.start))
        elif self.algorithm == "bfs2":
            self.bfs2()
        elif self.algorithm == "cus1":
            self.dls_search()
        elif self.algorithm == "cus2":
            self.ucs()
        else:
            print("\n< Invalid search algorithm. >\n")
            raise ValueError 
        