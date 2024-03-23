import numpy as np
import csv
from qc_scheduling import QCScheduling
from branch_and_bound import branch_and_bound_dfs
from grasp import launch

# Define problem data
NUMBER_OF_TASKS = 5
NUMBER_OF_QCS = 2
P = [8, 10, 20, 5, 30]    # Time to perform each Task
L = [1, 2, 3, 5, 6]       # Location of Task (value is ship-bay no.)
LC = [1, 4]               # Initial location of QCs (value is ship-bay no.)

# Define set of indices
# OMEGA: indices set of all tasks (default: from 0 -> NUMBER_OF_TASKS - 1)
# PSI: set of pairs of tasks that cannot be performed simultaneously (count from 0)
psi = {(1, 2), (2, 3), (3, 4), (4, 5)}
# PHI: set of ordered pairs of tasks between which there is a precedence relationship (count from 0)
phi = {}

def run_branch_and_bound():
    # qcs = QCScheduling(NUMBER_OF_TASKS, NUMBER_OF_QCS, P, L, LC, psi, phi)
    qcs = QCScheduling(
        6,
        2,
        [4, 8, 10, 5, 6, 7],
        [1, 1, 2, 3, 3, 4],
        [1, 4],
        {(1, 2)},
        {}
    )
    print('Running branch and bound...')
    branch_and_bound_dfs(qcs)

def run_grasp():
    """
    Main function.

    Try different alpha value to test which greedy level is the best. Call
    """

    experiment_number = 15  # run the experiment 30 times

    early_stop_list = []
    cost_list = []
    alpha = 0.1

    for early_stop in np.arange(50, 300, 50):
        temp_cost = []
        # run 30 time to calculate the mean cost of each alpha value
        for i in range(experiment_number):
            _, best_cost = launch(alpha, early_stop)
            temp_cost.append(best_cost)
        cost_mean = sum(temp_cost)/len(temp_cost)

        early_stop_list.append(early_stop)
        cost_list.append(cost_mean)

    # store the statistical data into a csv file
    with open('GRASP_result_early_stop.csv', 'w', newline='') as csv_file:
        writer = csv.writer(csv_file, delimiter=',')
        writer.writerow(['early_stop', 'cost'])
        for early_stop, cost in zip(early_stop_list, cost_list):
            writer.writerow([early_stop, cost])


def main():
    run_branch_and_bound()
    # run_grasp()

if __name__ == '__main__':
    main()
