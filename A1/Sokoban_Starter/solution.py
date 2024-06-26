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

# HELPER FUNCTIONS LIST
"""
    1. manhattan_distance(point1, point2)
    2. print_sokoban_state_attributes(state)
    3. at_storage(box_pos, storage_pos)
    4. on_edge(obj, state)
    5. storage_on_edge(box_position, state)
    6. is_in_tunnel_or_narrow_path(box, obstacles)
    7. is_in_tunnel_or_narrow_path_cached(box, obstacles)
    8. priority_distance(object, sets)
    9. priority_distance_exclude(object, storage_points, exclude)
    10. check_square_formation(boxes, state)
    11. check_adjacent_boxes_on_edge(box, state)
    12. check_adjacent_boxes_against_wall(box, state)
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

# Check if a box is at a corner position
def at_corner(box_pos, state):
    up = ((box_pos[0], box_pos[1] - 1) in state.obstacles) or (box_pos[1] - 1 < 0)
    down = ((box_pos[0], box_pos[1] + 1) in state.obstacles) or (box_pos[1] + 1 == state.height)
    left = ((box_pos[0] - 1, box_pos[1]) in state.obstacles) or (box_pos[0] - 1 < 0)
    right = ((box_pos[0] + 1, box_pos[1]) in state.obstacles) or (box_pos[0] + 1 == state.width)
    
    return (up or down) and (left or right)

# check if a box is at the edge of the wall
def on_edge(obj, state):
    return obj[0] == 0 or obj[0] == state.width - 1 or obj[1] == 0 or obj[1] == state.height - 1

# if no storage on the edge of the wall, this is a dead end
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

# check if a box is in a tunnel or narrow path
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

# Cache the results of the tunnel/narrow path check
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

# Priority queue for distance calculation
def priority_distance(object, sets):
    distances = []
    for things in sets:
        distance = manhattan_distance(object, things)
        # Push the distance and the box position as a tuple
        heapq.heappush(distances, (distance, things))
    # Pop the smallest distance
    min_distance, _ = heapq.heappop(distances)
   
    return min_distance

# Priority queue for distance calculation with exclusion
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

# Check for 2x2 square formation of boxes
def check_square_formation(boxes, state):
    for box in boxes:
        # Make sure 'box' is a tuple (x, y)
        right = (box[0] + 1, box[1])
        below = (box[0], box[1] + 1)
        diagonal = (box[0] + 1, box[1] + 1)

        # Check if 'right', 'below', and 'diagonal' boxes exist to form a square
        if right in boxes and below in boxes and diagonal in boxes:
            # Check if any of the square's positions are storage locations
            if not any(pos in state.storage for pos in [box, right, below, diagonal]):
                return True  # Unsolvable configuration
    
    return False  # No square formation found

# Check for 2x1 box deadlock
def check_adjacent_boxes_against_wall(box, state):
    # Horizontal adjacency check (box directly to the left or right)
    # Ensure both boxes are along the left or right edge and not at storage    
    if box[1] == 0 or box[1] == state.height - 1:
        if ((box[0] + 1, box[1]) in state.boxes or (box[0] - 1, box[1]) in state.boxes):
            if not (box in state.storage or (box[0], box[1] + 1) in state.storage or (box[0], box[1] - 1) in state.storage):
                return True

    # Vertical adjacency check (box directly above or below)
    # Ensure both boxes are along the top or bottom edge and not at storage
    if box[0] == 0 or box[0] == state.width - 1:
        if ((box[0], box[1] + 1) in state.boxes or (box[0], box[1] - 1) in state.boxes):
            if not (box in state.storage or (box[0] + 1, box[1]) in state.storage or (box[0] - 1, box[1]) in state.storage):
                return True
            
    return False  # No such configuration found

#--------------------------------------------------------------

# Initialize the cache at the global level
tunnel_path_cache = {}

# SOKOBAN HEURISTICS
def heur_alternate(state):
    # IMPLEMENT
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
            1.3 2x2 box deadlock
            1.4 2x1 box deadlock
            1.5 tunnel/narrow path penalty
        2. updates the Manhattan distance heuristic
            2.1 robot(s) to box(es)
            2.2 box(es) to storage(s)
    """
    
    # variables
    alt_val = 0
    tunnel_penalty = 1  # Define the penalty for being in a tunnel or narrow path
    used_storage = []  # Track storage points that have been paired

    # speeding up search to end
    for box_position in state.boxes:
        # check for corner deadlock
        if at_corner(box_position, state) and not at_storage(box_position, state):
            #print("COR works")
            #print(state.state_string())
            return math.inf
        
        # check for edge deadlock
        if on_edge(box_position, state) and not storage_on_edge(box_position, state):
            #print("EDG works")
            #print(state.state_string())
            return math.inf
        
        # Check for 2x2 square formation of boxes
        if check_square_formation(state.boxes, state):
            #print("SQR works")
            #print(state.state_string())
            return math.inf
        
        # check for 2x1 stuck
        if check_adjacent_boxes_against_wall(box_position, state):
            #print("STK works")
            #print(state.state_string())
            return math.inf
        
        # Apply tunnel/narrow path penalty
        if is_in_tunnel_or_narrow_path_cached(box_position, state.obstacles):
            alt_val += tunnel_penalty

    # each (available) robot(s) to its nearest box position
    for robot_position in state.robots:
        min_distance = priority_distance(robot_position, state.boxes)
        alt_val += (min_distance) / len(state.robots) # account for multiple robots

    # each box to its nearest storage point
    for box_position in state.boxes:
        # Calculate distance to the nearest available (unused) storage point
        min_distance, chosen_storage = priority_distance_exclude(box_position, state.storage, used_storage)
        if chosen_storage is not None:  # Ensure a storage was found and not all were excluded
            alt_val += min_distance - random.uniform(0, 0.69)
            used_storage.append(chosen_storage)

    return alt_val

def heur_manhattan_distance(state):
    # IMPLEMENT
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

    #print_sokoban_state_attributes(state)
    return total_distance  # CHANGE THIS

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search;
       an example of a trivial heuristic function
    '''
    return 0

def fval_function(sN, weight):
    # IMPLEMENT
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
    # IMPLEMENT 
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''
    
    # init
    target = None
    best_cost = (math.inf, math.inf, math.inf) # costbound definition in search egine
    # succ.gval > costbound[0] or 
    # succ_hval > costbound[1] or
    # succ.gval + succ_hval > costbound[2]

    se = SearchEngine('best_first', 'full')
    se.init_search(initial_state, sokoban_goal_state, heur_fn)
    
    end_time = os.times()[0] + timebound
    time_available = end_time - os.times()[0]

    # search
    while time_available > 0:
        # run the search engine
        final, stats = se.search(timebound=(time_available-0.1), costbound=best_cost) # 0.1s buffer
        time_available = end_time - os.times()[0] # check available time
        # check if goal state is found
        if final:
            best_cost = (final.gval, math.inf, math.inf) # <- prune based on the g-value (A1 handout) 
            target = final
        else:
            break

    return target, stats


def weighted_astar(initial_state, heur_fn, weight, timebound):
    # IMPLEMENT
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''
    
    # initialize search engine
    se = SearchEngine('custom', 'full')
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
    final, stats = se.search(timebound)

    return final, stats  # CHANGE THIS


def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT 
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''
    
    # init
    target = None
    best_cost = math.inf
    smaller_weight = 0.55
    end_time = os.times()[0] + timebound
    time_available = end_time - os.times()[0]

    # search
    while time_available > 0:
        # run weighted astar
        final, stats = weighted_astar(initial_state, heur_fn, weight, (time_available-0.1)) # 0.1s buffer
        weight = weight * smaller_weight # interatively smaller weight
        time_available = end_time - os.times()[0] # check available time
        # check if goal state is found
        if final:
            # check if a cost is better
            if final.gval < best_cost:
                best_cost = final.gval
                target = final
        else:
            break
    
    return target, stats

