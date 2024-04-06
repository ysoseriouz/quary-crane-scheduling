import sys
from pulp import PULP_CBC_CMD
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

# qcs = QCScheduling(NUMBER_OF_TASKS, NUMBER_OF_QCS, P, L, LC, psi, phi)
qcs = QCScheduling(
    25,
    3,
    [104, 278,166, 232, 254,265, 189, 352,108,227,400, 208, 136,173,206, 20,135,104, 365, 296,154,135,207,288, 361],
    [1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4],
    [1, 3, 4],
    {},
    {}
)

def displayResult(solution, filename):
    if solution:
        solution.lpModel.solve(PULP_CBC_CMD(msg=False))
        qcs.export(solution.lpModel, filename)
        print(f'Best solution ({solution.status()}): {solution.objective()}')
        print(solution)
    else:
        print('No solution found')

def run_branch_and_bound():
    print('Running branch and bound...')
    solution = branch_and_bound_dfs(qcs)
    displayResult(solution, filename='branch_and_bound')

def run_grasp():
    print('Running GRASP...')
    # TODO: try with different values of r and early_stop
    r = 0.4
    early_stop = 50
    solution = launch(qcs, r, early_stop)
    displayResult(solution, filename='grasp')


def main():
    run_branch_and_bound()
    print('\n--------------------------------------------------------------\n')
    run_grasp()

if __name__ == '__main__':
    try:
        option = int(sys.argv[1])
        if option == 1:
            run_branch_and_bound()
        elif option == 2:
            run_grasp()
        else:
            main()
    except IndexError:
        main()
