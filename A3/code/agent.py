"""
An AI player for Othello. 
"""

import random
import sys
import time

# You can use the functions in othello_shared to write your AI
from othello_shared import find_lines, get_possible_moves, get_score, play_move

def eprint(*args, **kwargs): #you can use this for debugging, as it will print to sterr and not stdout
    print(*args, file=sys.stderr, **kwargs)


############### UTILITY FUNCTION ###############
def compute_utility(board, current_player):
    p1_score, p2_score = get_score(board)
    return (p1_score - p2_score) if current_player == 1 else (p2_score - p1_score)


############### HEURISTIC ###############
def create_weight_matrix(size):
    # You can adjust these weights and expand the pattern for larger boards
    corner_weight = 100
    edge_weight = 10
    adjacent_corner_weight = -20
    inner_weight = 2

    # Initialize all weights to inner_weight first
    weights = [[inner_weight for _ in range(size)] for _ in range(size)]

    # Corners
    weights[0][0] = corner_weight
    weights[0][size-1] = corner_weight
    weights[size-1][0] = corner_weight
    weights[size-1][size-1] = corner_weight

    # Edges
    for i in range(1, size-1):
        weights[0][i] = edge_weight  # Top edge
        weights[i][0] = edge_weight  # Left edge
        weights[size-1][i] = edge_weight  # Bottom edge
        weights[i][size-1] = edge_weight  # Right edge

    # Adjacent to corners
    if size > 2:  # Adjacent corner weights are only relevant for boards larger than 2x2
        weights[1][0] = weights[0][1] = adjacent_corner_weight
        weights[1][size-1] = weights[0][size-2] = adjacent_corner_weight
        weights[size-2][0] = weights[size-1][1] = adjacent_corner_weight
        weights[size-2][size-1] = weights[size-1][size-2] = adjacent_corner_weight

    # The rest of the positions will keep the inner_weight.
    return weights

def compute_heuristic(board, current_player):
    board_size = len(board)
    
    if board_size > 4:
        weight_matrix = create_weight_matrix(board_size)
    elif board_size == 4:
        p1_score, p2_score = get_score(board)
        return (p1_score - p2_score) if current_player == 1 else (p2_score - p1_score)
    
    utility = 0

    for i in range(board_size):
        for j in range(board_size):
            if board[i][j] == current_player:
                utility += weight_matrix[i][j]
            elif board[i][j] != 0:  # Assuming '0' is an empty space
                utility -= weight_matrix[i][j]

    return utility


############### CACHING ###############
cached_states = {}


############################### MINIMAX ###############################
def minimax_max_node(board, current_player, limit, caching = 0): #returns highest possible utility
    # Check if the state is in the cache
    global cached_states
    if caching and (board in cached_states):
        return cached_states[board]
    
    # computes the utility assuming it is your turn to claim land
    max_utility = float('-inf')
    best_move = None
    possible_moves = get_possible_moves(board, current_player)

    if len(possible_moves) == 0 or (limit == 0):  # Check if the game is over
        return None, compute_utility(board, current_player)
    
    for move in possible_moves:
        new_board = play_move(board, current_player, move[0], move[1])
        _, utility = minimax_min_node(new_board, current_player, limit-1, caching)
        
        if caching:
            cached_states[new_board] = (move, utility)
        
        if utility > max_utility:
            max_utility = utility
            best_move = move
    
    return best_move, max_utility

def minimax_min_node(board, current_player, limit, caching = 0):
    # Check if the state is in the cache
    global cached_states
    if caching and (board in cached_states):
        return cached_states[board]
    
    #  computes the utility assuming it is your opponent’s turn to claim lan
    opponent = 2 if current_player == 1 else 1
    
    min_utility = float('inf')
    best_move = None
    possible_moves = get_possible_moves(board, opponent)

    if len(possible_moves) == 0 or (limit == 0):  # Check if the game is over
        return None, compute_utility(board, current_player)

    for move in possible_moves:
        new_board = play_move(board, opponent, move[0], move[1])
        _, utility = minimax_max_node(new_board, current_player, limit-1, caching)
        
        if caching:
            cached_states[new_board] = (move, utility)
        
        if utility < min_utility:
            min_utility = utility
            best_move = move
    
    return best_move, min_utility

def select_move_minimax(board, current_player, limit, caching = 0):
    """
    Given a board and a player color, decide on a move. 
    The return value is a tuple of integers (i,j), where
    i is the column and j is the row on the board.  

    Note that other parameters are accepted by this function:
    If limit is a positive integer, your code should enfoce a depth limit that is equal to the value of the parameter.
    Search only to nodes at a depth-limit equal to the limit.  If nodes at this level are non-terminal return a heuristic 
    value (see compute_utility)
    If caching is ON (i.e. 1), use state caching to reduce the number of state evaluations.
    If caching is OFF (i.e. 0), do NOT use state caching to reduce the number of state evaluations.    
    """
    cached_states.clear()
    best_move, _ = minimax_max_node(board, current_player, limit, caching)
    return best_move


############### ORDERING ###############
def sort_moves_by_utility(board, possible_moves, current_player, reverse_flag):
    # Generate a list of tuples with (move, utility) --> stays with compute_utility 
    move_utilities = [(move, compute_utility(play_move(board, current_player, move[0], move[1]), current_player)) for move in possible_moves]

    # Sort the list of tuples based on utility, which is the second item in each tuple
    move_utilities.sort(key=lambda x: x[1], reverse=reverse_flag)

    # Return only the sorted moves, excluding the utilities
    sorted_moves = [move for move, utility in move_utilities]
    return sorted_moves


############################### ALPHA-BETA PRUNING ###############################
def alphabeta_max_node(board, current_player, alpha, beta, limit, caching = 0, ordering = 0):    
    # Check if the state is in the cache
    global cached_states
    if caching and (board, current_player, alpha, beta) in cached_states:
        return cached_states[(board, current_player, alpha, beta)]

    max_utility = float('-inf')
    best_move = None
    possible_moves = get_possible_moves(board, current_player)

    if len(possible_moves) == 0 or (limit == 0):  # Check if the game is over
        return None, compute_utility(board, current_player)

    if ordering:
        possible_moves = sort_moves_by_utility(board, possible_moves, current_player, reverse_flag=True) # in descending order

    for move in possible_moves:
        new_board = play_move(board, current_player, move[0], move[1])
        _, utility = alphabeta_min_node(new_board, current_player, alpha, beta, limit-1, caching, ordering)
        
        if caching:
            cached_states[(new_board, current_player, alpha, beta)] = (move, utility)
        
        if utility > max_utility:
            max_utility = utility
            best_move = move
        
        alpha = max(alpha, utility)
        if beta <= alpha:
            break

    return best_move, max_utility

def alphabeta_min_node(board, current_player, alpha, beta, limit, caching = 0, ordering = 0):    
    # Computes the utility assuming it is your opponent’s turn to claim land
    opponent = 2 if current_player == 1 else 1
    
    # Check if the state is in the cache
    global cached_states
    if caching and (board, current_player, alpha, beta) in cached_states:
        return cached_states[(board, current_player, alpha, beta)]

    min_utility = float('inf')
    best_move = None
    possible_moves = get_possible_moves(board, opponent)

    if len(possible_moves) == 0 or (limit == 0):  # Check if the game is over
        return None, compute_utility(board, current_player)

    #if ordering: # tester --> speed will be slower if min node also uses ordering
    #    possible_moves = sort_moves_by_utility(board, possible_moves, opponent, reverse_flag=False) # in ascending order

    for move in possible_moves:
        new_board = play_move(board, opponent, move[0], move[1])
        _, utility = alphabeta_max_node(new_board, current_player, alpha, beta, limit-1, caching, ordering)
        
        if caching:
            cached_states[(new_board, current_player, alpha, beta)] = (move, utility)
        
        if utility < min_utility:
            min_utility = utility
            best_move = move
        
        beta = min(beta, utility)
        if beta <= alpha:
            break
    
    return best_move, min_utility

def select_move_alphabeta(board, current_player, limit, caching = 0, ordering = 0):
    """
    Given a board and a player color, decide on a move. 
    The return value is a tuple of integers (i,j), where
    i is the column and j is the row on the board.  

    Note that other parameters are accepted by this function:
    If limit is a positive integer, your code should enfoce a depth limit that is equal to the value of the parameter.
    Search only to nodes at a depth-limit equal to the limit.  If nodes at this level are non-terminal return a heuristic 
    value (see compute_utility)
    If caching is ON (i.e. 1), use state caching to reduce the number of state evaluations.
    If caching is OFF (i.e. 0), do NOT use state caching to reduce the number of state evaluations.    
    If ordering is ON (i.e. 1), use node ordering to expedite pruning and reduce the number of state evaluations. 
    If ordering is OFF (i.e. 0), do NOT use node ordering to expedite pruning and reduce the number of state evaluations. 
    """
    cached_states.clear()
    pos_infty = float('inf')
    neg_infty = float('-inf')
    best_move, _ = alphabeta_max_node(board, current_player, neg_infty, pos_infty, limit, caching, ordering)
    return best_move


###############################################################################################
def run_ai():
    """
    This function establishes communication with the game manager.
    It first introduces itself and receives its color.
    Then it repeatedly receives the current score and current board state
    until the game is over.
    """
    print("Othello AI") # First line is the name of this AI
    arguments = input().split(",")
    
    color = int(arguments[0]) #Player color: 1 for dark (goes first), 2 for light. 
    limit = int(arguments[1]) #Depth limit
    minimax = int(arguments[2]) #Minimax or alpha beta
    caching = int(arguments[3]) #Caching 
    ordering = int(arguments[4]) #Node-ordering (for alpha-beta only)

    if (minimax == 1): eprint("Running MINIMAX")
    else: eprint("Running ALPHA-BETA")

    if (caching == 1): eprint("State Caching is ON")
    else: eprint("State Caching is OFF")

    if (ordering == 1): eprint("Node Ordering is ON")
    else: eprint("Node Ordering is OFF")

    if (limit == -1): eprint("Depth Limit is OFF")
    else: eprint("Depth Limit is ", limit)

    if (minimax == 1 and ordering == 1): eprint("Node Ordering should have no impact on Minimax")

    while True: # This is the main loop
        # Read in the current game status, for example:
        # "SCORE 2 2" or "FINAL 33 31" if the game is over.
        # The first number is the score for player 1 (dark), the second for player 2 (light)
        next_input = input()
        status, dark_score_s, light_score_s = next_input.strip().split()
        dark_score = int(dark_score_s)
        light_score = int(light_score_s)

        if status == "FINAL": # Game is over.
            print
        else:
            board = eval(input()) # Read in the input and turn it into a Python
                                  # object. The format is a list of rows. The
                                  # squares in each row are represented by
                                  # 0 : empty square
                                  # 1 : dark disk (player 1)
                                  # 2 : light disk (player 2)
            #eprint("board ", board) --> input is a board in form of lists of list
            tuple_of_tuples = tuple(tuple(inner_list) for inner_list in board)
            #eprint("tuple_of_tuples ", tuple_of_tuples) --> needs to be in form of tuple of tuples
            # Select the move and send it to the manager
            if (minimax == 1): #run this if the minimax flag is given
                movei, movej = select_move_minimax(tuple_of_tuples, color, limit, caching)
                #eprint("movei: ", movei, "movej: ", movej)
            else: #else run alphabeta
                movei, movej = select_move_alphabeta(tuple_of_tuples, color, limit, caching, ordering)
            
            print("{} {}".format(movei, movej))

if __name__ == "__main__":
    run_ai()
    
