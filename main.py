import sys, os
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

# TODO: try with different values of r and early_stop
# GRASP settings
r = 0.4
early_stop = 50


def displayResult(solution, filename):
    if solution:
        if solution.lpModel is not None:
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
    solution = launch(qcs, r, early_stop)
    displayResult(solution, filename='grasp')

def exportSolution(solution, filename, dirname):
    if solution is not None and solution.lpModel is not None:
        solution.lpModel.solve(PULP_CBC_CMD(msg=False))
        qcs.export(solution.lpModel, filename, dirname)

def process(dirname):
    # Create directory to store results
    rootDir = 'output'
    dirpath = os.path.join(rootDir, dirname)
    if not os.path.exists(rootDir):
        os.makedirs(rootDir)
    if not os.path.exists(dirpath):
        os.makedirs(dirpath)

    s1, r1 = branch_and_bound_dfs(qcs)
    s2, r2 = launch(qcs, r, early_stop)
    exportSolution(s1, 'branch_and_bound', dirpath)
    exportSolution(s2, 'grasp', dirpath)

    with open(os.path.join(dirpath, 'output.txt'), 'w') as f:
        f.write(f'DATA INPUT {dirname}\n')
        f.write(f'Number of tasks   : {qcs.num_tasks}\n')
        f.write(f'Number of QCs     : {qcs.num_qcs}\n')
        f.write(f'Tasks duration    : {qcs.task_durations}\n')
        f.write(f'Tasks location    : {qcs.task_locations}\n')
        f.write(f'QCs location      : {qcs.qc_locations}\n')
        f.write(f'PSI               : {qcs.non_simultaneous_tasks}\n')
        f.write(f'PHI               : {qcs.precedence_constrained_tasks}\n')

        f.write('\nOUTPUT:\n')
        f.write('Branch and bound_DFS\n')
        if s1:
            f.write(str(s1) + '\n')
            f.write(f'Objective value: {s1.objective()}\n')
        else:
            f.write('No solution found\n')
        f.write(f'Run time: {r1}\n')
        
        f.write('\nGRASP\n')
        if s2:
            f.write(str(s2) + '\n')
            f.write(f'Objective value: {s2.objective()}\n')
        else:
            f.write('No solution found\n')
        f.write(f'Run time: {r2}\n')


def main():
    run_branch_and_bound()
    print('\n--------------------------------------------------------------\n')
    run_grasp()

if __name__ == '__main__':
    process(sys.argv[1])
