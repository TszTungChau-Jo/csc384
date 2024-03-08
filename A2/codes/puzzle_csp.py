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
from itertools import product
import operator
import math

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

# helpers
def get_min_unique_rows_or_columns(cage_vars): # working as expected
    row_indices = set()
    col_indices = set()
    
    for var in cage_vars:
        # Directly accessing the name of the Variable object
        name = var.name
        # Assuming variable names are in the format "Vxy"
        row_index, col_index = int(name[1]), int(name[2])
        
        row_indices.add(row_index)
        col_indices.add(col_index)
    
    # The minimum number of unique rows or columns is the smaller of the two sets' sizes
    min_unique = min(len(row_indices), len(col_indices))
    return min_unique

def generate_combinations_with_repetition(n, length, repetition):
    """
    Generate combinations of numbers from 1 to n, each number can repeat up to 'repetition' times.
    """
    return [combination for combination in product(range(1, n + 1), repeat=length)
            if all(combination.count(x) <= repetition for x in set(combination))]

def modified_satisfying_tuples_for_subtraction(size, cage_vars, target, min_unique):
    """
    Generate satisfying tuples considering the minimum number of unique rows or columns.
    """
    combinations = generate_combinations_with_repetition(size, len(cage_vars), min_unique)
    
    # For subtraction, only permutations of these combinations are considered
    satisfying_tuples = []
    for combination in combinations:
        for perm in itertools.permutations(combination):
            if perm[0] - sum(perm[1:]) == target:
                satisfying_tuples.append(perm)
                #break  # Once a valid permutation is found, no need to check others -- tbd
    
    unique_set = set(t for t in satisfying_tuples)
    satisfying_tuples = [tuple(t) for t in unique_set]    

    return satisfying_tuples

def modified_satisfying_tuples_for_division(size, cage_vars, target, min_unique):
    """
    Generate satisfying tuples considering the minimum number of unique rows or columns.
    """
    combinations = generate_combinations_with_repetition(size, len(cage_vars), min_unique)
    
    satisfying_tuples = []
    for combination in combinations:
        # Check all permutations of the combination
        for perm in itertools.permutations(combination):
            # Perform division from left to right
            result = perm[0]
            for num in perm[1:]:
                if num == 0:  # Avoid division by zero
                    break
                result /= num

            # If the result of the division equals the target, add the permutation to the satisfying tuples
            if result == target:
                satisfying_tuples.append(perm)
    
    unique_set = set(t for t in satisfying_tuples)
    satisfying_tuples = [tuple(t) for t in unique_set]    

    return satisfying_tuples

"""
The three encodings take as input a valid FunPuzz grid, which is a list of lists, 
where the first list has a single element, N, which is the size of each dimension of the board, 
and each following list represents a cage in the grid.

Cell names are encoded as integers in the range 11,...,nn
and each inner list contains the numbers of the cells that are included in the corresponding cage, 
followed by the target value for that cage and the operation (0 = `+`, 1 = `-`, 2 = `/`, 3 = `*`).

Examples:
For the fpuzz_grid ((3), (11,12,13,6,0), (21,22,31,2,2), ....) corresponds to a 3x3 board where
1. cells 11, 12 and 13 must sum to 6, and
2. the result of dividing some permutation of cells 21, 22, and 31 must be 2. 
That is, (C21/C22)/C23 = 2 or (C21/C23)/C22 = 2, or (C22/C21)/C23 = 2, etc...


Note: If a list has two elements,
the first element corresponds to a cell, 
and the second one—the target—is the value enforced on that cell.
"""

def caged_csp(fpuzz_grid):
    # The size of the grid is the first element in the list
    size = fpuzz_grid[0][0]
    
    # Create a list of lists to hold the variable objects
    var_array = [[Variable(f'V{i}{j}', domain=list(range(1, size+1))) 
                  for j in range(1, size+1)] for i in range(1, size+1)]
    
    # Flatten the list of lists to pass to the CSP object
    vars_flat = [var for sublist in var_array for var in sublist]
    csp = CSP(f'FunPuzz-{size}x{size}', vars_flat)
    
    # Implement binary_ne constraints
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

    # Implement cage constraints
    for cage in fpuzz_grid[1:]:
        if len(cage) == 2:
            # Handle a single cell with a fixed value through a unary constraint
            cell, target = cage
            var = var_array[(cell//10)-1][(cell%10)-1]
            constraint_name = f'Fixed-{cell}'
            constraint = Constraint(constraint_name, [var])
            satisfying_tuples = [(target,)]  # The only tuple that satisfies this constraint is the target value itself
            constraint.add_satisfying_tuples(satisfying_tuples)
            csp.add_constraint(constraint)
        else:
            target = cage[-2]
            operation = cage[-1]
            cage_vars = [var_array[(cell//10)-1][(cell%10)-1] for cell in cage[:-2]]
            
            constraint = Constraint(f'Cage-{cage[:-2]}', cage_vars)
            satisfying_tuples = []
            
            min_unique = get_min_unique_rows_or_columns(cage_vars)
            #print(f"Minimum number of unique rows or columns: {min_unique}")
            #print(cage_vars)

            # Generate satisfying tuples based on the operation
            if operation == 0:  # Addition
                satisfying_tuples = [t for t in itertools.product(range(1, size+1), repeat=len(cage_vars)) 
                                    if sum(t) == target]
                # make sure to remove duplicates
                unique_set = set(t for t in satisfying_tuples)
                satisfying_tuples = [tuple(t) for t in unique_set]
                #print("add sat:  ", satisfying_tuples)
            
            elif operation == 1:  # Subtraction; updated -> should be fine?
                min_unique = get_min_unique_rows_or_columns(cage_vars)
                satisfying_tuples = modified_satisfying_tuples_for_subtraction(size, cage_vars, target, min_unique)
                
                all_permutations = [perm for tup in satisfying_tuples for perm in itertools.permutations(tup)]
                satisfying_tuples = all_permutations
                
                # make sure to remove duplicates
                unique_set = set(t for t in satisfying_tuples)
                satisfying_tuples = [tuple(t) for t in unique_set]  
                
                #print("sub sat:  ", satisfying_tuples)
            
            elif operation == 2:  # Division; updated -> should be fine?
                min_unique = get_min_unique_rows_or_columns(cage_vars)
                
                satisfying_tuples = modified_satisfying_tuples_for_division(size, cage_vars, target, min_unique)
                #satisfying_tuples = [t for t in itertools.permutations(range(1, size+1), len(cage_vars)) 
                #                    if t[0] / max(1, functools.reduce(operator.mul, t[1:], 1)) == target 
                #                    or max(1, functools.reduce(operator.mul, t[1:], 1)) / t[0] == target]
                
                all_permutations = [perm for tup in satisfying_tuples for perm in itertools.permutations(tup)]
                satisfying_tuples = all_permutations
                
                # make sure to remove duplicates
                unique_set = set(t for t in satisfying_tuples)
                satisfying_tuples = [tuple(t) for t in unique_set]  
                #print("div sat:  ", satisfying_tuples)
            
            elif operation == 3:  # Multiplication
                satisfying_tuples = [t for t in itertools.product(range(1, size+1), repeat=len(cage_vars)) 
                                    if math.prod(t) == target]
                # make sure to remove duplicates
                unique_set = set(t for t in satisfying_tuples)
                satisfying_tuples = [tuple(t) for t in unique_set]  
                #print("mul sat:  ", satisfying_tuples)

            constraint.add_satisfying_tuples(satisfying_tuples)
            csp.add_constraint(constraint)
    
    return csp, var_array
