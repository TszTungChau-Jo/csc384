#Look for #IMPLEMENT tags in this file.
'''
All encodings need to return a CSP object, and a list of lists of Variable objects 
representing the board. The returned list of lists is used to access the 
solution. 

For example, after these three lines of code

    csp, var_array = caged_csp(board)
    solver = BT(csp)
    solver.bt_search(prop_FC, var_ord)

var_array[0][0].get_assigned_value() should be the correct value in the top left
cell of the FunPuzz puzzle.

The grid-only encodings do not need to encode the cage constraints.

1. binary_ne_grid (worth 10/100 marks)
    - An enconding of a FunPuzz grid (without cage constraints) built using only 
      binary not-equal constraints for both the row and column constraints.

2. nary_ad_grid (worth 10/100 marks)
    - An enconding of a FunPuzz grid (without cage constraints) built using only n-ary 
      all-different constraints for both the row and column constraints. 

3. caged_csp (worth 25/100 marks) 
    - An enconding built using your choice of (1) binary binary not-equal, or (2) 
      n-ary all-different constraints for the grid.
    - Together with FunPuzz cage constraints.

'''

from cspbase import *
import itertools

def binary_ne_grid(fpuzz_grid):
    # The size of the grid is the first element in the list
    size = fpuzz_grid[0][0]
    
    # Create a list of lists to hold the variable objects
    var_array = [[Variable(f'V{i}{j}', domain=list(range(1, size+1))) 
                  for j in range(1, size+1)] for i in range(1, size+1)]
    
    # Flatten the list of lists to pass to the CSP object
    vars_flat = [var for sublist in var_array for var in sublist]
    csp = CSP(f'FunPuzz-{size}x{size}', vars_flat)
    
    # Add constraints
    for i in range(size):
        for j in range(size):
            # Row constraints
            for k in range(j+1, size):
                csp.add_constraint(Constraint(f'Row{i}{j}{k}', [var_array[i][j], var_array[i][k]]))
            
            # Column constraints
            for k in range(i+1, size):
                csp.add_constraint(Constraint(f'Col{i}{j}{k}', [var_array[i][j], var_array[k][j]]))
    
    # Each constraint needs to have its satisfying tuples generated and added
    for con in csp.get_all_cons():
        tuples = []
        for val1 in range(1, size+1):
            for val2 in range(1, size+1):
                if val1 != val2:
                    tuples.append((val1, val2))
        con.add_satisfying_tuples(tuples)

    return csp, var_array

def nary_ad_grid(fpuzz_grid):
    size = fpuzz_grid[0][0]  # The size of the grid is the first element in the list
    
    # Initialize the CSP object
    csp = CSP(f'FunPuzz-{size}x{size}')
    
    # Create a 2D list (matrix) of Variable objects representing the grid
    var_array = [[Variable(f'V{i+1}{j+1}', domain=list(range(1, size+1))) 
                  for j in range(size)] for i in range(size)]
    
    # Add all variables to the CSP
    for row in var_array:
        for var in row:
            csp.add_var(var)
    
    # Add n-ary all-different constraints for rows and columns
    for i in range(size):
        row_vars = var_array[i]
        row_constraint = Constraint(f'Row-{i+1}-AllDiff', row_vars)
        # Generate and add satisfying tuples for the row constraint
        row_constraint.add_satisfying_tuples(
            [tuple for tuple in itertools.permutations(range(1, size + 1))])
        csp.add_constraint(row_constraint)
        
    for j in range(size):
        col_vars = [var_array[i][j] for i in range(size)]
        col_constraint = Constraint(f'Col-{j+1}-AllDiff', col_vars)
        # Generate and add satisfying tuples for the column constraint
        col_constraint.add_satisfying_tuples(
            [tuple for tuple in itertools.permutations(range(1, size + 1))])
        csp.add_constraint(col_constraint)
    
    return csp, var_array


def caged_csp(fpuzz_grid):
    ##IMPLEMENT 
    return