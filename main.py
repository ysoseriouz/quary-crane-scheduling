from pulp import LpStatus, PULP_CBC_CMD
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
    6,
    2,
    [4, 8, 10, 5, 6, 7],
    [1, 1, 2, 3, 3, 4],
    [1, 4],
    {(1, 2)},
    {}
)

def displayResult(solution, filename):
    if solution:
        solution.lpModel.solve(PULP_CBC_CMD(msg=1))
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
    early_stop = 100
    solution = launch(qcs, r, early_stop)
    displayResult(solution, filename='grasp')


def main():
    run_branch_and_bound()
    print('\n--------------------------------------------------------------\n')
    run_grasp()

if __name__ == '__main__':
    main()
