from time import time
from pulp import *
from search import *
import util
from qc_scheduling import QCScheduling, QCState

# Define problem data
NUMBER_OF_TASKS = 5
NUMBER_OF_QCS = 2
P = [8, 10, 20, 5, 30]    # Time to perform each Task
L = [1, 2, 3, 5, 6]       # Location of Task (value is ship-bay no.)
LC = [0, 4]               # Initial location of QCs (value is ship-bay no.)

# Define set of indices
# OMEGA: indices set of all tasks (default: from 0 -> NUMBER_OF_TASKS - 1)
# PSI: set of pairs of tasks that cannot be performed simultaneously (count from 0)
psi = {(0, 1), (1, 2), (2, 3), (3, 4)}
# PHI: set of ordered pairs of tasks between which there is a precedence relationship (count from 0)
phi = {}


def branch_and_bound_dfs():
    qcs = QCScheduling(NUMBER_OF_TASKS, NUMBER_OF_QCS, P, L, LC, psi, phi)

    fringe = util.Stack()
    explored = set()
    start_node = Node(state=qcs.getStartState(), parent=None, action=None)
    fringe.push(start_node)
    solutions = []
    best_objective = float('inf')
    first_feasible = False
    count = 0

    start_time = time()
    while not fringe.isEmpty():
        count += 1
        node = fringe.pop()
        node.state.lpModel.solve()
        curr_objective = node.state.lpModel.objective.value()

        # Feasible solution found
        if node.state.lpModel.status == LpStatusOptimal:
            # Last level
            if qcs.isGoalState(node.state):
                explored.add(node.state)
                curr_objective = node.state.objective()
                if curr_objective < best_objective:
                    best_objective = curr_objective
                    qcs.addObjectiveUpBound(node.state, best_objective)
                    solutions.append(node)
                continue
        
        # Branching
        feasible_child_nodes = []
        if node.state not in explored:
            explored.add(node.state)
            for child, action, cost in qcs.expand(node.state):
                if violate_constraints_8(qcs, child):
                    explored.add(child)
                    continue
                
                if child not in explored:
                    child_node = Node(state=child, parent=node, action=action, 
                                        depth=node.depth + 1, cost=node.cost + cost)
                    feasible_child_nodes.append(child_node)

        # TODO: wtf is this even mean
        delete_dominated_nodes(qcs, feasible_child_nodes, explored)

        # Calculate lower bound and prune sub-tree
        for child_node in feasible_child_nodes:
            if qcs.computeLowBound(child_node.state) > best_objective:
                explored.add(child_node.state)
            else:
                fringe.push(child_node)

    if len(solutions) > 0:
        best_sol = None
        for sol in solutions:
            if sol.state.objective() <= best_objective:
                best_sol = sol
        
        print('Number of solutions: ', len(solutions))
        print('Best solution: ', best_objective)
        print(best_sol.state)
    else:
        print('No solution found')
    print(f'Done in {time() - start_time}(s) with {count}(iters)')

def violate_constraints_8(qcs, state):
    task_completion_time_map = qcs.getTaskCompletionTime(state)
    for i, j in phi:
        if i not in task_completion_time_map or j not in task_completion_time_map:
            return False
        
        Di = task_completion_time_map[i]
        Dj = task_completion_time_map[j]
        if Di + qcs.task_durations[j] > Dj:
            return True
    
    return False

def delete_dominated_nodes(qcs, nodes, explored):
    pass
