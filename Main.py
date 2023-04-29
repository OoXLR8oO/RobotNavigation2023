from collections import deque
from Agent import *
from Maze import *
from Cell import *

import sys

arg_textfile = sys.argv[1]
arg_searcher = sys.argv[2]


obstacles = []
dimensions = []
goals = []
start = []

# Reads data from a file using an indexer
def ReadLineValues(s, index):
    if index == 0: # Eg. [5,11]
        dimensions.extend([int(num) for num in s.strip().strip('[]').split(',')])
        return dimensions
    
    elif index == 1: # Eg. (0,1)
        start.extend([int(num.strip('()')) for num in s.split(',')])
        return start
    
    elif index == 2:
        if "(" in s:
            if "|" in s: # Eg. (7,0) | (10,3)
                goal_strs = s.split("|") 
                for goal_str in goal_strs:
                    x, y = map(int, goal_str.strip()[1:-1].split(","))
                    goals.append([x, y])
            else:
                x, y = map(int, s.strip()[1:-1].split(","))
                start.extend([x, y])
        return goals
    
    elif index > 2:
        if s.count(',') == 3: # Eg. (2, 0, 2, 2) ...
            obs = [int(num.strip('()')) for num in s.split(',')]
            obstacles.append(obs)
            return obs

def SetMaze():
    # Create maze and set goals, start and obstacles
    # print("D: ",  dimensions)
    # print("S: ", start)
    # print("G: ", goals)
    # print("O: ", obstacles)
    maze = Maze(dimensions[1], dimensions[0], start, goals, obstacles)
    maze.set_start()
    maze.set_goals()
    maze.set_obstacles()
    return maze

def main():
    if len(sys.argv) != 3:
        print(len(sys.argv))
        print("ERROR: Incorrect number of arguments.")
        input("Press any key to quit...")
        sys.exit()
    print("< COS30019 Robot Navigation >")
    
    # File I/O                    
    try:
        with open(arg_textfile, 'r', encoding='utf-8') as file:
            print("Kartik Punna (103609675)\n")
            line_num = 0
            for line in file:
                line = line.strip()
                ReadLineValues(line, line_num)
                line_num += 1
        file.close()
    except ValueError:
        print("ERROR: Invalid inputs.")
        input("Press any key to quit...")
        sys.exit()
    
    maze = SetMaze()
    #maze.draw_maze() # Print Maze to Terminal

    try:
        # Attempt to create agent instance
        agent = Agent(maze, arg_searcher.lower())
        
    except ValueError: # If incorrect algorithm is entered, catch exception and let while loop reset
        print("ERROR: Selected Algorithm is invalid.")
        input("Press any key to quit...")
        sys.exit()
    agent.find_path()
    agent.draw_path()

    maze.draw_maze() # Draw search results (Uncomment for visual representation)
    print(arg_textfile, agent.algorithm, len(agent.nodes))
    if agent.goal_found:
        for dir in agent.get_directions():
            print(dir, end=" ")
    else:
        print("No solution found.")
    
    input("\nPress enter to quit...")
    sys.exit()

if __name__ == '__main__':
    main()
