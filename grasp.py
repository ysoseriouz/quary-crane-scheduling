# Ref: https://github.com/angrymushroom/GRASP

import random
from time import time

MAX_ITERATION = 1000
TIME_LIMIT = 10800 # 3 hours

def launch(qcs, alpha, early_stop):
    count = 0
    start_time = time()

    best_cost = float('inf')
    best_sol = None

    while count < MAX_ITERATION:
        count += 1
        if count % 100 == 0:
            print('ITERATION %d' % count)
            if time() - start_time >= TIME_LIMIT:
                print('Time limit exceeded')
                break
        new_sol = construct_greedy_solution(qcs, alpha)
        if new_sol is None:
            continue
        new_sol = local_search(new_sol, early_stop, qcs)

        if new_sol.objective() < best_cost and new_sol.isFeasible():
            best_cost = new_sol.objective()
            best_sol = new_sol
            print(f'(ITERATION {count}) New solution found: {best_cost}')
            print(best_sol)

    run_time = time() - start_time
    print(f'Done in {run_time}(s) with {count}(iters)')
    return best_sol, run_time


def construct_greedy_solution(qcs, alpha):
    state = qcs.getStartState(False)
    while not qcs.isGoalState(state):
        state = qcs.expandGrasp(state, alpha)
        if state is None:
            return None

    return state


def local_search(sol, early_stop, qcs):
    cost = sol.objective()

    for qc in range(qcs.num_qcs):
        count = 0
        while count < early_stop:
            count += 1
            new_sol = stochastic_swap(sol, qc)  # randomly swap two edges to explore the possible neighbors.
            if new_sol is None:
                break

            new_sol.evaluateGrasp(qcs)
            new_cost = new_sol.objective()   # calculate the total cost of a solution

            #  update the solution and cost if a better solution is found
            if new_cost < cost and new_sol.isFeasible():
                print('*', end='', flush=True)
                sol = new_sol
                cost = new_cost
                count = 0
            else:
                del new_sol

    return sol


def stochastic_swap(sol, qc):
    sol_size = len(sol.qc_assigned_tasks[qc])
    # cannot swap if there are less than 2 tasks -> skip QC
    if sol_size < 2:
        return None
    
    sol_copy = sol.clone()
    indices = list(range(sol_size))

    index1 = random.choice(indices)
    indices.remove(index1)
    index2 = random.choice(indices)

    temp_task = sol_copy.qc_assigned_tasks[qc][index1]
    sol_copy.qc_assigned_tasks[qc][index1] = sol_copy.qc_assigned_tasks[qc][index2]
    sol_copy.qc_assigned_tasks[qc][index2] = temp_task

    return sol_copy
