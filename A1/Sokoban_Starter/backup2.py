"""
17/22 version -- when lucky up to 18/22
"""
#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
import random  # for tie-breaking
import heapq  # for priority queue
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
    print("Sokoban State Attributes:")
    print(f"Width: {state.width}")
    print(f"Height: {state.height}")
    print(f"Robots: {state.robots}")
    print(f"Boxes: {state.boxes}")
    print(f"Storage: {state.storage}")
    print(f"Obstacles: {state.obstacles}")
    print(state.state_string())
    return None

# Check if a box is at a storage position
def at_storage(box_pos, state):
    return (box_pos in state.storage)

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

# Check if a box is at a corner position and not at a storage position
def at_corner(box_pos, state):
    blocking_left = ((box_pos[0] - 1, box_pos[1]) in state.obstacles) or (box_pos[0] - 1 < 0)
    blocking_right = ((box_pos[0] + 1, box_pos[1]) in state.obstacles) or (box_pos[0] + 1 == state.width)
    blocking_up = ((box_pos[0], box_pos[1] - 1) in state.obstacles) or (box_pos[1] - 1 < 0)
    blocking_down = ((box_pos[0], box_pos[1] + 1) in state.obstacles) or (box_pos[1] + 1 == state.height)
    
    return (blocking_left or blocking_right) and (blocking_up or blocking_down)

# check if a box is at the edge of the wall
def on_edge(obj, state):
    return obj[0] == 0 or obj[0] == state.width - 1 or obj[1] == 0 or obj[1] == state.height - 1

#if no storage on the edge of the wall, this is a dead end
def storage_on_edge(box_position, state): # box already on edge
    for storage in state.storage:
        if box_position[0] == 0 and storage[0] == box_position[0]:
            return True
        elif box_position[0] == state.width - 1 and storage[0] == box_position[0]:
            return True
        elif box_position[1] == 0 and storage[1] == box_position[1]:
            return True
        elif box_position[1] == state.height - 1 and storage[1] == box_position[1]:
            return True
    return False

def is_path_blocked(start, end, boxes):
    if start[0] == end[0]:  # Same column
        step = 1 if start[1] < end[1] else -1
        for y in range(start[1] + step, end[1], step):
            if (start[0], y) in boxes:
                return True
    elif start[1] == end[1]:  # Same row
        step = 1 if start[0] < end[0] else -1
        for x in range(start[0] + step, end[0], step):
            if (x, start[1]) in boxes:
                return True
    else:  # Not a straight line; for simplicity, consider it blocked
        return True
    return False

def priority_distance(object, sets):
    distances = []
    for box_position in sets:
        distance = manhattan_distance(object, box_position)
        # Push the distance and the box position as a tuple
        heapq.heappush(distances, (distance, box_position))
    # Pop the smallest distance
    min_distance, _ = heapq.heappop(distances)
    return min_distance

def is_direct_path_blocked(box, storage, boxes):
    # Simplified check for a direct path blockage
    if box[0] == storage[0]:  # Same column
        for b in boxes:
            if b != box and b[0] == box[0] and ((b[1] > box[1] and b[1] < storage[1]) or (b[1] < box[1] and b[1] > storage[1])):
                return True
    elif box[1] == storage[1]:  # Same row
        for b in boxes:
            if b != box and b[1] == box[1] and ((b[0] > box[0] and b[0] < storage[0]) or (b[0] < box[0] and b[0] > storage[0])):
                return True
    return False

def is_in_tunnel_or_narrow_path(box, obstacles):
    # Checks around the box
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # Up, Down, Left, Right
    obstacle_count = 0

    for dx, dy in directions:
        adjacent = (box[0] + dx, box[1] + dy)
        if adjacent in obstacles:
            obstacle_count += 1

    # A box is considered to be in a tunnel or narrow path if there are obstacles in at least two opposite directions
    # This simplified check assumes that being adjacent to any two obstacles constitutes a tunnel/narrow path scenario
    # This is a basic approximation and might not perfectly capture all tunnel situations, especially for complex puzzles
    return obstacle_count >= 2

def is_in_tunnel_or_narrow_path_cached(box, obstacles):
    # Create a unique key for the current configuration
    # Note: The key should uniquely represent the box position and the state of its surroundings
    # For simplicity, we'll just use the box position here, but consider including relevant obstacle information for more complex scenarios
    key = (box, tuple(sorted(obstacles)))

    # Check if the result is in the cache
    if key in tunnel_path_cache:
        return tunnel_path_cache[key]

    # If not cached, perform the check
    is_tunnel = is_in_tunnel_or_narrow_path(box, obstacles)  # Use the original function or implement the logic here

    # Store the result in the cache before returning
    tunnel_path_cache[key] = is_tunnel

    return is_tunnel

def is_frozen_box(box, state):
    """
    Check if a box is frozen. A box is frozen if it's surrounded in such a way that it cannot move.
    This is a simplified version; more complex checks may involve recursive checks on adjacent boxes.
    """
    # Check immediate surroundings for walls or other boxes
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
    frozen = True
    for d in directions:
        adjacent = (box[0] + d[0], box[1] + d[1])
        # If any adjacent space is free (not a wall or a box), the box is not frozen
        if adjacent not in state.obstacles and adjacent not in state.boxes:
            frozen = False
            break
    return frozen

def check_for_deadlock_pattern(box, state):
    # Check horizontal tunnel deadlock
    if check_horizontal_tunnel_deadlock(box, state):
        return True
    # Check vertical tunnel deadlock
    if check_vertical_tunnel_deadlock(box, state):
        return True
    return False

def check_horizontal_tunnel_deadlock(box, state):
    # Check if there's a wall directly to the left or right, indicating a potential tunnel
    left_wall = (box[0] - 1, box[1]) in state.obstacles or box[0] - 1 < 0
    right_wall = (box[0] + 1, box[1]) in state.obstacles or box[0] + 1 >= state.width
    if not left_wall and not right_wall:
        return False  # Not a horizontal tunnel

    # Check if the tunnel leads to a storage location
    for storage in state.storage:
        if storage[1] == box[1]:  # Same row, indicating a potential path to storage
            return False  # There's a storage in this row, so it's not a dead-end tunnel
    
    # No storage in the tunnel's row, indicating a dead-end tunnel
    return True

def check_vertical_tunnel_deadlock(box, state):
    # Check if there's a wall directly above or below, indicating a potential tunnel
    up_wall = (box[0], box[1] - 1) in state.obstacles or box[1] - 1 < 0
    down_wall = (box[0], box[1] + 1) in state.obstacles or box[1] + 1 >= state.height
    if not up_wall and not down_wall:
        return False  # Not a vertical tunnel

    # Check if the tunnel leads to a storage location
    for storage in state.storage:
        if storage[0] == box[0]:  # Same column, indicating a potential path to storage
            return False  # There's a storage in this column, so it's not a dead-end tunnel

    # No storage in the tunnel's column, indicating a dead-end tunnel
    return True

def calculate_dynamic_weights(state):
    boxes_at_storage = sum(at_storage(box, state) for box in state.boxes)
    total_boxes = len(state.boxes)
    progress = boxes_at_storage / total_boxes if total_boxes else 0

    # Adjust these thresholds as needed based on testing and optimization
    if progress < 0.5:
        return 0.7, 0.3  # Prioritize path clearing early on
    else:
        return 0.3, 0.7  # Prioritize box-to-storage placement later

def priority_distance_exclude(object, storage_points, exclude):
    distances = []
    for storage_position in storage_points:
        if storage_position in exclude:
            continue  # Skip storage points that have been paired/used
        distance = manhattan_distance(object, storage_position)
        heapq.heappush(distances, (distance, storage_position))
    if distances:
        min_distance, chosen_storage = heapq.heappop(distances)
        return min_distance, chosen_storage
    return None, None  # In case all storage points are excluded or list is empty


#--------------------------------------------------------------

# Initialize the cache at the global level or as part of your class
tunnel_path_cache = {}

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
    #map_size = state.width * state.height

    alt_val = 0
    penalty_per_blocked_box = 2  # Define a fixed penalty for each blocked path
    tunnel_penalty = 1  # Define the penalty for being in a tunnel or narrow path
    #weight_path_clearing, weight_box_placement = calculate_dynamic_weights(state) # Calculate dynamic weights -- worsen performance
    used_storage = []  # Track storage points that have been paired

    # speeding up search to end
    for box_position in state.boxes:
        # check for corner deadlock
        if at_corner(box_position, state) and not at_storage(box_position, state):
            #print("COR works")
            #print_sokoban_state_attributes(state)
            return math.inf
        
        # check for edge deadlock
        if on_edge(box_position, state) and not storage_on_edge(box_position, state):
            #print("EDG works")
            #print_sokoban_state_attributes(state)
            return math.inf
        
        # Advanced deadlock detection -- worsen performance
        #if is_frozen_box(box_position, state) or check_for_deadlock_pattern(box_position, state):
        #    return math.inf  # Indicate an unsolvable state
        
        # Check for paths blocked by other boxes
        for storage_position in state.storage:
            if is_direct_path_blocked(box_position, storage_position, state.boxes):
                alt_val += penalty_per_blocked_box
                break  # No need to check other storages if one path is already found to be blocked

        # Apply tunnel/narrow path penalty
        #if is_in_tunnel_or_narrow_path(box_position, state.obstacles):
        if is_in_tunnel_or_narrow_path_cached(box_position, state.obstacles):
            alt_val += tunnel_penalty

    # each (available) robot(s) to its nearest box position
    for robot_position in state.robots:
        min_distance = priority_distance(robot_position, state.boxes)
        alt_val += (min_distance) / num_robot # account for multiple robots

    # each box to its nearest storage point
    for box_position in state.boxes:

        # Calculate distance to the nearest available (unused) storage point
        min_distance, chosen_storage = priority_distance_exclude(box_position, state.storage, used_storage)
        if chosen_storage is not None:  # Ensure a storage was found and not all were excluded
            alt_val += min_distance - random.uniform(0, 0.69)
            used_storage.append(chosen_storage)  # Mark this storage as used

    return alt_val


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
    

    total_distance = 0
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

    print_sokoban_state_attributes(state)
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
    return (sN.gval + weight*sN.hval) # our f(n) = g(n) + w*h(n)

#--------------------------------------------------------------

# SEARCH ALGORITHMS
def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT 4th
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''
    
    
    return None, None #CHANGE THIS


def weighted_astar(initial_state, heur_fn, weight, timebound):
    # IMPLEMENT 5th
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''
    
    # 直接用 manhanntan distance 
    
    return None, None  # CHANGE THIS


def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT 6th
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''
    
    
    return None, None #CHANGE THIS

