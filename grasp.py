# Ref: https://github.com/angrymushroom/GRASP

import random
from time import time


MAX_ITERATION = 1000

def launch(qcs, alpha, early_stop):
    count = 0
    start_time = time()

    best_cost = float('inf')
    best_sol = None

    while count < MAX_ITERATION:
        count += 1
        if count % 100 == 0:
            print('ITERATION %d' % count)
        new_sol = construct_greedy_solution(qcs, alpha)
        new_sol = local_search(new_sol, early_stop, qcs)

        if new_sol.objective() < best_cost and new_sol.isFeasible():
            best_cost = new_sol.objective()
            best_sol = new_sol
            print(f'(ITERATION {count}) New solution found: {best_cost}')
            print(best_sol)

    print(f'Done in {time() - start_time}(s) with {count}(iters)')
    return best_sol


def construct_greedy_solution(qcs, alpha):
    state = qcs.getStartState()
    while not qcs.isGoalState(state):
        state = qcs.expandGrasp(state, alpha)

    return state


def local_search(sol, early_stop, qcs):
    explored = set()
    cost = sol.objective()

    explored.add(sol)
    for qc in range(qcs.num_qcs):    
        count = 0
        while count < early_stop:
            new_sol = stochastic_swap(sol, qc)  # randomly swap two edges to explore the possible neighbors.
            if new_sol in explored:
                count += 1
            else:
                explored.add(new_sol)
                new_sol.evaluateGrasp(qcs)
                new_cost = new_sol.objective()   # calculate the total cost of a solution

                #  update the solution and cost if a better solution is found
                if new_cost < cost and new_sol.isFeasible():
                    print(new_sol)
                    sol = new_sol
                    cost = new_cost
                    count = 0
                else:
                    count += 1

    return sol


def stochastic_swap(sol, qc):
    sol_copy = sol.clone()
    sol_size = len(sol_copy.qc_assigned_tasks[qc])
    if sol_size < 2:
        return sol_copy
    
    indices = list(range(sol_size))

    index1 = random.choice(indices)
    indices.remove(index1)
    index2 = random.choice(indices)

    temp_task = sol_copy.qc_assigned_tasks[qc][index1]
    sol_copy.qc_assigned_tasks[qc][index1] = sol_copy.qc_assigned_tasks[qc][index2]
    sol_copy.qc_assigned_tasks[qc][index2] = temp_task

    return sol_copy
