import numpy as np
import csv
from branch_and_bound import branch_and_bound_dfs
from grasp import launch

def run_branch_and_bound():
    branch_and_bound_dfs()

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
