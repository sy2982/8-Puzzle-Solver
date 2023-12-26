from __future__ import division
from __future__ import print_function

import sys
import math
import time
import queue as Q
import resource

## The Class that Represents the Puzzle
class PuzzleState(object):
    """
        The PuzzleState stores a board configuration and implements
        movement instructions to generate valid children.
    """
    def __init__(self, config, n, parent=None, action="Initial", cost=0):
        """
        :param config->List : Represents the n*n board, for e.g. [0,1,2,3,4,5,6,7,8] represents the goal state.
        :param n->int : Size of the board
        :param parent->PuzzleState
        :param action->string
        :param cost->int
        """
        if n*n != len(config) or n < 2:
            raise Exception("The length of config is not correct!")
        if set(config) != set(range(n*n)):
            raise Exception("Config contains invalid/duplicate entries : ", config)

        self.n        = n
        self.cost     = cost
        self.parent   = parent
        self.action   = action
        self.config   = config
        self.children = []

        # Get the index and (row, col) of empty block
        self.blank_index = self.config.index(0)

    def display(self):
        """ Display this Puzzle state as a n*n board """
        for i in range(self.n):
            print(self.config[3*i : 3*(i+1)])

    def move_up(self):
        """ 
        Moves the blank tile one row up.
        :return a PuzzleState with the new configuration
        """

        # Blank tile can only move up if in the 2nd and 3rd row
        if self.blank_index > 2:
            temp = list(self.config)

            # Swapping with -3 to move up in a 3x3 board
            temp[self.blank_index] = temp[self.blank_index - 3]
            temp[self.blank_index - 3] = 0
            return PuzzleState(temp, self.n, self, "'Up'", self.cost + 1)
        
        return None
      
    def move_down(self):
        """
        Moves the blank tile one row down.
        :return a PuzzleState with the new configuration
        """

        # Blank tile can only move down if in 1st and 2nd row
        if self.blank_index < 6:
            temp = list(self.config)

            # Swapping with +3 to move down in a 3x3 board
            temp[self.blank_index] = temp[self.blank_index + 3]
            temp[self.blank_index + 3] = 0
            return PuzzleState(temp, self.n, self, "'Down'", self.cost + 1)
        
        return None
      
    def move_left(self):
        """
        Moves the blank tile one column to the left.
        :return a PuzzleState with the new configuration
        """

        # Blank tile can only move left if in 2nd and 3rd column
        if self.blank_index % 3 > 0:
            temp = list(self.config)

            # Swapping with -1 to move down in a 3x3 board
            temp[self.blank_index] = temp[self.blank_index - 1]
            temp[self.blank_index - 1] = 0
            return PuzzleState(temp, self.n, self, "'Left'", self.cost + 1)
        
        return None

    def move_right(self):
        """
        Moves the blank tile one column to the right.
        :return a PuzzleState with the new configuration
        """

        # Blank tile can only move left if in 1st and 2nd column
        if self.blank_index % 3 < 2:
            temp = list(self.config)

            # Swapping with +1 to move down in a 3x3 board
            temp[self.blank_index] = temp[self.blank_index + 1]
            temp[self.blank_index + 1] = 0
            return PuzzleState(temp, self.n, self, "'Right'", self.cost + 1)
        
        return None
      
    def expand(self):
        """ Generate the child nodes of this node """
        
        # Node has already been expanded
        if len(self.children) != 0:
            return self.children
        
        # Add child nodes in order of UDLR
        children = [
            self.move_up(),
            self.move_down(),
            self.move_left(),
            self.move_right()]

        # Compose self.children of all non-None children states
        self.children = [state for state in children if state is not None]
        return self.children

# Function that Writes to output.txt
def write_output(path_to_goal, cost_of_path, nodes_expanded, search_depth, max_search_depth, running_time, max_ram_usage):
    with open("output.txt", 'wt') as file:
        file.write(f"path_to_goal: [{(', '.join(path_to_goal))}]\n")
        file.write(f"cost_of_path: {cost_of_path}\n")
        file.write(f"nodes_expanded: {nodes_expanded}\n")
        file.write(f"search_depth: {search_depth}\n")
        file.write(f"max_search_depth: {max_search_depth}\n")
        file.write(f"running_time: {running_time:.8f}\n")
        file.write(f"max_ram_usage: {max_ram_usage:.8f}\n")

def bfs_search(initial_state):
    """BFS search"""

    start_time = time.time()
    start_ram = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss

    nodes_expanded = 0
    max_search_depth = 0

    frontier = Q.Queue()
    explored = set()
    frontier.put(initial_state)

    while not frontier.empty():

        state = frontier.get()
        explored.add(tuple(state.config))

        # Test if at goal, then send to output if true
        if test_goal(state):
            path_to_goal = []
            while state.parent:
                path_to_goal.insert(0, state.action)
                state = state.parent

            end_ram = (resource.getrusage(resource.RUSAGE_SELF).ru_maxrss - start_ram) / (2 ** 20)
            write_output(path_to_goal, len(path_to_goal), len(explored) -1 , len(path_to_goal), max_search_depth, time.time() - start_time, end_ram)
            return True

        # Check for neighbors
        for neighbor in state.expand():        
            if tuple(neighbor.config) not in explored:
                frontier.put(neighbor)
                if neighbor.cost > max_search_depth:
                    max_search_depth = neighbor.cost

        nodes_expanded += 1

    return False

def dfs_search(initial_state):
    """DFS search"""

    start_time = time.time()
    start_ram = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss

    nodes_expanded = 0
    max_search_depth = 0

    frontier = Q.LifoQueue()
    explored = set()
    frontier.put(initial_state)

    while not frontier.empty():

        state = frontier.get()
        explored.add(tuple(state.config))
        nodes_expanded += 1
        
        # Test if at goal, then send to output if true
        if test_goal(state):
            path_to_goal = []
            while state.parent:
                path_to_goal.insert(0, state.action)
                state = state.parent

            end_ram = (resource.getrusage(resource.RUSAGE_SELF).ru_maxrss - start_ram) / (2 ** 20)
            write_output(path_to_goal, len(path_to_goal), nodes_expanded - 1, len(path_to_goal), max_search_depth, time.time() - start_time, end_ram)
            return True

        # Check for neighbors
        for neighbor in state.expand()[::-1]:
            if tuple(neighbor.config) not in explored:
                frontier.put(neighbor)
                explored.add(tuple(neighbor.config))
                if neighbor.cost > max_search_depth:
                    max_search_depth = neighbor.cost

    return False

def A_star_search(initial_state):
    """A * search"""

    start_time = time.time()
    start_ram = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss

    nodes_expanded = 0
    max_search_depth = 0

    frontier = []
    explored = set()

    # Add estimated cost and state to the frontier
    frontier.append((calculate_total_cost(initial_state), initial_state))

    while frontier:

        # Sort frontier by estimated cost to make a priority queue
        frontier.sort(key=lambda x: x[0])
        state = frontier.pop(0)[1]

        explored.add(tuple(state.config))
        nodes_expanded += 1

        # Test if at goal, then send to output if true
        if test_goal(state):
            path_to_goal = []
            while state.parent:
                path_to_goal.insert(0, state.action)
                state = state.parent

            end_ram = (resource.getrusage(resource.RUSAGE_SELF).ru_maxrss - start_ram) / (2 ** 20)
            write_output(path_to_goal, len(path_to_goal), nodes_expanded - 1, len(path_to_goal), len(path_to_goal), time.time() - start_time, end_ram)
            return True

        # Check for neighbors
        for neighbor in state.expand():
            if tuple(neighbor.config) not in explored:
                g = state.cost + 1
                h = calculate_total_cost(neighbor)
                f = g + h
                if neighbor not in [node[1] for node in frontier]:
                    frontier.append((f, neighbor))

    return False

def calculate_total_cost(state):
    """calculate the total estimated cost of a state"""

    # Heuristic: h1 = Misplaced tiles and h2 = Manhattan distance
    h1 = sum(1 for i, tile in enumerate(state.config) if tile != i)
    h2 = sum(calculate_manhattan_dist(i, tile, state.n) for i, tile in enumerate(state.config))

    return h1 + h2

def calculate_manhattan_dist(idx, value, n):
    """calculate the manhattan distance of a tile"""

    # If already at goal
    if value == 0:
        return 0
    
    # Find coordinates for manhattan distance
    x1, y1 = divmod(value, n)
    x2, y2 = divmod(idx, n) 

    return abs(x1 - x2) + abs(y1 - y2)


def test_goal(puzzle_state):
    """test the state is the goal state or not"""

    # Goal state for puzzle
    return puzzle_state.config == [0,1,2,3,4,5,6,7,8]

# Main Function that reads in Input and Runs corresponding Algorithm
def main():
    search_mode = sys.argv[1].lower()
    begin_state = sys.argv[2].split(",")
    begin_state = list(map(int, begin_state))
    board_size  = int(math.sqrt(len(begin_state)))
    hard_state  = PuzzleState(begin_state, board_size)
    start_time  = time.time()
    
    if   search_mode == "bfs": bfs_search(hard_state)
    elif search_mode == "dfs": dfs_search(hard_state)
    elif search_mode == "ast": A_star_search(hard_state)
    else: 
        print("Enter valid command arguments !")
        
    end_time = time.time()
    print("Program completed in %.3f second(s)"%(end_time-start_time))

if __name__ == '__main__':
    main()
