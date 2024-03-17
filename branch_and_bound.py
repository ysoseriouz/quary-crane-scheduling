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

    while not fringe.isEmpty():
        node = fringe.pop()
        node.state.lpModel.solve(PULP_CBC_CMD(msg=False))
        solved_objective = node.state.lpModel.objective.value()

        # Feasible solution found
        if node.state.lpModel.status == LpStatusOptimal:
            # First feasible solution
            if not first_feasible:
                print('First feasible solution: ', solved_objective)
                first_feasible = True

            # Last level
            if qcs.isGoalState(node.state):
                if solved_objective < best_objective:
                    best_objective = solved_objective
                    qcs.add_objective_upperbound(node.state, best_objective)
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
            if qcs.lower_bound(child_node.state) > best_objective:
                explored.add(child_node.state)
            else:
                fringe.push(child_node)

    print('Number of solutions: ', len(solutions))
    best_sol = None
    best_objective = float('inf')
    if len(solutions) > 0:
        for sol in solutions:
            total_completion_time = max(qcs.get_qc_completion_time(sol.state).values())
            if total_completion_time < best_objective:
                best_objective = total_completion_time
                best_sol = sol
        
        print('Best solution: ', best_objective)
        print(best_sol.state)
    else:
        print('No solution found')

def violate_constraints_8(qcs, state):
    task_completion_time_map = qcs.get_task_completion_time(state)
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
