#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete problem solution.  

'''This file will contain different constraint propagators to be used within 
   bt_search.

   propagator == a function with the following template
      propagator(csp, newly_instantiated_variable=None)
           ==> returns (True/False, [(Variable, Value), (Variable, Value) ...]

      csp is a CSP object---the propagator can use this to get access
      to the variables and constraints of the problem. The assigned variables
      can be accessed via methods, the values assigned can also be accessed.

      newly_instaniated_variable is an optional argument.
      if newly_instantiated_variable is not None:
          then newly_instantiated_variable is the most
           recently assigned variable of the search.
      else:
          progator is called before any assignments are made
          in which case it must decide what processing to do
           prior to any variables being assigned. SEE BELOW

       The propagator returns True/False and a list of (Variable, Value) pairs.
       Return is False if a deadend has been detected by the propagator.
       in this case bt_search will backtrack
       return is true if we can continue.

      The list of variable values pairs are all of the values
      the propagator pruned (using the variable's prune_value method). 
      bt_search NEEDS to know this in order to correctly restore these 
      values when it undoes a variable assignment.

      NOTE propagator SHOULD NOT prune a value that has already been 
      pruned! Nor should it prune a value twice

      PROPAGATOR called with newly_instantiated_variable = None
      PROCESSING REQUIRED:
        for plain backtracking (where we only check fully instantiated 
        constraints) 
        we do nothing...return true, []

        for forward checking (where we only check constraints with one
        remaining variable)
        we look for unary constraints of the csp (constraints whose scope 
        contains only one variable) and we forward_check these constraints.


      PROPAGATOR called with newly_instantiated_variable = a variable V
      PROCESSING REQUIRED:
         for plain backtracking we check all constraints with V (see csp method
         get_cons_with_var) that are fully assigned.

         for forward checking we forward check all constraints with V
         that have one unassigned variable left

   '''

def prop_BT(csp, newVar=None):
    '''Do plain backtracking propagation. That is, do no 
    propagation at all. Just check fully instantiated constraints'''
    
    if not newVar:
        return True, []
    for c in csp.get_cons_with_var(newVar):
        if c.get_n_unasgn() == 0:
            vals = []
            vars = c.get_scope()
            for var in vars:
                vals.append(var.get_assigned_value())
            if not c.check(vals):
                return False, []
    return True, []

def prop_FC(csp, newVar=None):
    '''Do forward checking. That is check constraints with 
       only one uninstantiated variable. Remember to keep 
       track of all pruned variable, value pairs and return '''
    
    """
    Forward Checking (FC) constraint propagator - this function should take CSP object and a variable, 
    check constraints that have exactly one uninstantiated variable in their scope, and prune appropriately. 
    If a dead-end was found return tuple (False, list), else : return (True,list), 
    where list is a list of (Variable, value) tuples that have been pruned by the propagator.
    """

    # Check if forward checking is triggered by a variable assignment
    if newVar is None:
        # Forward checking is not applicable before any assignments
        return True, []

    pruned = []  # To keep track of pruned values
    # Iterate over all constraints that include newVar
    for constraint in csp.get_cons_with_var(newVar):
        # Check if there's exactly one unassigned variable in the constraint's scope
        if constraint.get_n_unasgn() == 1:
            unassigned_var = constraint.get_unasgn_vars()[0]
            for value in unassigned_var.cur_domain():
                # Temporarily assign the value to check if the constraint is satisfied
                if not constraint.has_support(unassigned_var, value):
                    # If no support, prune the value and add to pruned list
                    unassigned_var.prune_value(value)
                    pruned.append((unassigned_var, value))
                    # If the domain of the unassigned variable is empty, return failure
                    if unassigned_var.cur_domain_size() == 0:
                        return False, pruned

    return True, pruned


def prop_FI(csp, newVar=None):
    '''Do full inference. If newVar is None we initialize the queue
       with all variables.'''

    pruned = []  # To keep track of pruned values
    if newVar is None:
        varQueue = [v for v in csp.get_all_vars() if not v.is_assigned()]
    else:
        varQueue = [newVar]

    while varQueue:
        W = varQueue.pop(0)
        for C in csp.get_cons_with_var(W):
            for V in set(C.get_scope()) - {W}:
                S = V.cur_domain()
                for d in list(S):  # Make a copy of the current domain to iterate over
                    if not C.has_support(V, d):
                        V.prune_value(d)
                        pruned.append((V, d))
                        if V.cur_domain_size() == 0:
                            # Domain wipe out, fail immediately
                            return False, pruned
                        else:
                            if V not in varQueue: # if V not already in queue
                                varQueue.append(V)

    return True, pruned

