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
    
# Method to compute utility value of terminal state; defined to be the number of hectares they own minus the number of hectares their competitor ow
def compute_utility(board, current_player):
    p1_score, p2_score = get_score(board)
    return (p1_score - p2_score) if current_player == 1 else (p2_score - p1_score)

# Method to check if the current player have any possible move
def is_no_more_move(board, current_player):
    if get_possible_moves(board, current_player):
        return False
    return True

# Better heuristic value of board
def compute_heuristic(board, current_player): #not implemented, optional
    #IMPLEMENT
    return 0 #change this!

############ MINIMAX ############################### Ignore the limit, and caching parameters for now.
def minimax_max_node(board, current_player, limit, caching = 0): #returns highest possible utility
    # computes the utility assuming it is your turn to claim land

    max_utility = float('-inf')
    best_move = None
    possible_moves = get_possible_moves(board, current_player)

    if is_no_more_move(board, current_player):  # Check if the game is over
        return None, compute_utility(board, current_player)
    
    for move in possible_moves:
        new_board = play_move(board, current_player, move[0], move[1])
        #eprint(new_board)
        _, utility = minimax_min_node(new_board, current_player, limit, caching)
        
        if utility > max_utility:
            max_utility = utility
            best_move = move
    return best_move, max_utility

def minimax_min_node(board, current_player, limit, caching = 0):
    #  computes the utility assuming it is your opponent’s turn to claim lan
    opponent = 2 if current_player == 1 else 1
    
    min_utility = float('inf')
    best_move = None
    possible_moves = get_possible_moves(board, opponent)

    if is_no_more_move(board, opponent):  # Check if the game is over
        return None, compute_utility(board, current_player)

    for move in possible_moves:
        new_board = play_move(board, opponent, move[0], move[1])
        _, utility = minimax_max_node(new_board, current_player, limit, caching)
        
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
    best_move, _ = minimax_max_node(board, current_player, limit, caching)
    return best_move


############ ALPHA-BETA PRUNING #####################
def alphabeta_min_node(board, current_player, alpha, beta, limit, caching = 0, ordering = 0):
    #IMPLEMENT (and replace the line below)

    return ((0,0),0) #change this!

def alphabeta_max_node(board, current_player, alpha, beta, limit, caching = 0, ordering = 0):
    #IMPLEMENT (and replace the line below)

    return ((0,0),0) #change this!

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
    #IMPLEMENT (and replace the line below)

    return (0,0) #change this!

####################################################
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

            # Select the move and send it to the manager
            if (minimax == 1): #run this if the minimax flag is given
                movei, movej = select_move_minimax(board, color, limit, caching)
                #eprint("movei: ", movei, "movej: ", movej)
            else: #else run alphabeta
                movei, movej = select_move_alphabeta(board, color, limit, caching, ordering)
            
            print("{} {}".format(movei, movej))

if __name__ == "__main__":
    run_ai()
