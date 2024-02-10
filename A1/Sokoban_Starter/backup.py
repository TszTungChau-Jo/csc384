"""
14/22 version
"""

#   Look for #IMPLEMENT tags in this file. These tags indicate what has 
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

# HELPER FUNCTIONS
"""
    1. manhattan_distance(point1, point2)
    2. print_sokoban_state_attributes(state)
    3. is_box_at_storage(box_pos, storage_pos)
"""

# Calculate Manhattan distances (x1, y1) and (x2, y2) => |x1 - x2| + |y1 - y2|
def manhattan_distance(point1, point2):
    """Calculate the Manhattan distance between two points"""
    return abs(point1[0] -point2[0]) + abs(point1[1] - point2[1])

# Testing configuration
def print_sokoban_state_attributes(state):
    print("\nSokoban State Attributes:")
    print(f"Width: {state.width}")
    print(f"Height: {state.height}")
    print(f"Robots: {state.robots}")
    print(f"Boxes: {state.boxes}")
    print(f"Storage: {state.storage}")
    print(f"Obstacles: {state.obstacles}")
    print(state.state_string())
    return None

# Check if a box is at a storage position
def is_box_at_storage(box_pos, storage):
    return (box_pos in storage)

# Check if a box is surrounded by obstacles on 2 adjacent sides
def is_box_surrounded_by_obstacles(box_pos, state):
    if box_pos == (0, 0):
        return True
    elif box_pos == (0, state.width-1):
        return True
    elif box_pos == (state.height-1, 0):
        return True
    elif box_pos == (state.height-1, state.width-1):
        return True
    else:
        return False

#if no storage on the edge of the wall, this is a dead end
def no_storage_on_edge(box_position, state): # box already on edge
    for storage in state.storage:
        if (box_position[0] == 0) and (storage[0] == 0):
            return False
        elif (box_position[0] == state.width-1) and (storage[0] == state.width-1):
            return False
        elif (box_position[1] == 0) and (storage[1] == 0):
            return False
        elif (box_position[1] == state.height-1) and (storage[1] == state.height-1):
            return False
    
    return True

#--------------------------------------------------------------

# SOKOBAN HEURISTICS
def heur_alternate(state):
    # IMPLEMENT 2nd
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.
    # EXPLAIN YOUR HEURISTIC IN THE COMMENTS. Please leave this function (and your explanation) at the top of your solution file, to facilitate marking.
    """
    Better heuristic:
        1. check for deadlocks
            1.1 corner deadlocks: 4 corner positions -> ostacles on 2 adjacent sides
            1.2 wall(edge) deadlocks: along the edge of a wall wil no storage point
        2. updates the Manhattan distance heuristic by (robot(s) to boxes) and (boxes to storage)   
            2.1 robot(s) to box(es)
            2.2 box(es) to storage(s)
    """
    # variables
    num_boxes = len(state.boxes)
    num_robot = len(state.robots)
    num_storage = len(state.storage)
    num_obstacles = len(state.obstacles)
    x_bound = state.width - 1
    y_bound = state.height - 1
    homeless_boxes = []
    movable_boxes = []

    #print_sokoban_state_attributes(state)
    # boxes not in storage -- note: worse than before
    """
    for box_position in state.boxes:
        if not is_box_at_storage(box_position, state.storage):
            homeless_boxes.append(box_position)
    """

    #print(f"Homeless Boxes: {homeless_boxes}")
    
    """
    # movable boxes -> no obstacles/robot/boxes around that restrict the movment | free to move
    blocked = 0
    for box_position in state.boxes:
        x, y = box_position

        #print(f"Box Position: {box_position}")
        
        if (((x - 1, y) in state.obstacles or (x - 1, y) in state.robots or (x - 1, y) in state.boxes or (x-1<0)) and 
            ((x, y - 1) in state.obstacles or (x, y - 1) in state.robots or (x, y - 1) in state.boxes or (y-1<0))):
            blocked = 1
        elif(((x + 1, y) in state.obstacles or (x + 1, y) in state.robots or (x + 1, y) in state.boxes or (x+1>x_bound)) and 
             ((x, y - 1) in state.obstacles or (x, y - 1) in state.robots or (x, y - 1) in state.boxes or (y-1<0))):
            blocked = 1
        elif(((x - 1, y) in state.obstacles or (x - 1, y) in state.robots or (x - 1, y) in state.boxes or (x-1<0)) and 
             ((x, y + 1) in state.obstacles or (x, y + 1) in state.robots or (x, y + 1) in state.boxes or (y+1>y_bound))):
            blocked = 1
        elif(((x + 1, y) in state.obstacles or (x + 1, y) in state.robots or (x + 1, y) in state.boxes or (x+1>x_bound)) and 
             ((x, y + 1) in state.obstacles or (x, y + 1) in state.robots or (x, y + 1) in state.boxes or (y+1>y_bound))):
            blocked = 1
        else:
            movable_boxes.append(box_position)
    """
    

    #print("Movable boxes:", movable_boxes)

    # 1 check for deadlocks
    # 1.1 corner deadlocks
    
    # movable boxes -> no obstacles/robot/boxes around that restrict the movment | free to move
    """
    for box_position in state.boxes:
        x, y = box_position
        #print(f"Box Position: {box_position}")
        if (x-1<0) and (y-1<0):
            return math.inf
        elif (x+1>x_bound) and (y-1<0):
            return math.inf
        elif (x-1<0) and (y+1>y_bound):
            return math.inf
        elif(x+1>x_bound) and (y+1>y_bound):
            return math.inf
    """
    
    # 1.2 wall(edge) deadlocks

    
    # 2 updates the Manhattan distance heuristic by (robot(s) to boxes) and (boxes to storage)
    alt_val = 0
    # 2.1 each (available) robot to its nearest box position
    for robot_position in state.robots:
        smallest_distance = math.inf
        for box_position in state.boxes:
            distance = manhattan_distance(robot_position, box_position)
            if distance < smallest_distance:
                smallest_distance = distance
        # append for each robot
        alt_val += (smallest_distance) / num_robot # account for multiple robots

    # 2.2 each box to its nearest storage point
    for box_position in state.boxes:
        smallest_distance = math.inf
        for storage_position in state.storage:
            distance = manhattan_distance(box_position, storage_position)
            if distance < smallest_distance:
                smallest_distance = distance
        # append for each box
        alt_val += smallest_distance

    # add penalty for each box that is not in storage
    #alt_val += len(homeless_boxes) * 10
    
    return alt_val  # CHANGE THIS


def heur_manhattan_distance(state):
    # IMPLEMENT 1st
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.
    
    """
    Notes:
        goal: estimates how many moves a current state is from a goal state
        side: Ignore the positions of obstacles in your calculations and assume that 
              many boxes can be stored at one location
    """

    # variables
    total_distance = 0
    num_boxes = len(state.boxes)
    num_robot = len(state.robots)
    num_storage = len(state.storage)
    num_obstacles = len(state.obstacles)

    # iterate through each box position
    for box_position in state.boxes:
        min_distance = math.inf # initialize min_distance to infinity
        # find the closest storage position
        for storage_position in state.storage:
            distance = manhattan_distance(box_position, storage_position)
            if distance < min_distance:
                min_distance = distance
        # add the minimum distance to the total distance
        total_distance += min_distance

    #print_sokoban_state_attributes(state)
    #print(f"self-Total Distance: {total_distance}")
    #print(f"num boxes: {num_boxes}, num robot: {num_robot}, num storage: {num_storage}")
    return total_distance  # CHANGE THIS

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search;
       an example of a trivial heuristic function
    '''
    return 0


def fval_function(sN, weight):
    # IMPLEMENT 3rd
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    
    
    return 0 #CHANGE THIS


# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
    # IMPLEMENT 4th
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''
    
    
    return None, None  # CHANGE THIS


def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT 5th
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''
    
    
    return None, None #CHANGE THIS


def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT 6th
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''
    
    
    return None, None #CHANGE THIS



