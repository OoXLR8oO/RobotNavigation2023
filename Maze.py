from Cell import *

class Maze:
    def __init__(self, width, height, start, goals, obstacles):
        self.width = width
        self.height = height
        self.start = [start[1], start[0]]
        self.obstacles = obstacles
        self.goals = [[goal[1], goal[0]] for goal in goals]
        self.cells = [[Cell(x, y, 0) for y in range(width)] for x in range(height)]
    
    def get_cell(self, row, col):
        return self.cells[row][col]

    def set_obstacles(self): # cells[h][w]
        # Turn all Obstacle tiles from 0 to 1
        for coords in self.obstacles:
            for w in range(coords[2]):
                for h in range(coords[3]):
                    # 0 = Open, 1 = Obstacle
                    self.cells[coords[1] + h][coords[0] + w].type = 1

    def set_start(self):
        self.get_cell(*self.start).type = 7

    def set_goals(self):
        goal_cells = []
        for goal in self.goals:
            self.get_cell(goal[0], goal[1]).type = 2
            goal_cells.append(self.get_cell(goal[0], goal[1]))
        self.goals = goal_cells
        

    def draw_maze(self):
        print("")
        for row in self.cells:
            line = ""
            for cell in row:
                line += str(cell.type) + " "
            print(line)