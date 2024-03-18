from pulp import *
from search import SearchProblem

class QCState:
    def __init__(self, num_qcs, lpModel = None):
        self.qc_assigned_tasks = [[] for _ in range(num_qcs)]
        self.qc_completion_time = [0] * num_qcs
        self.lpModel = lpModel

    def __eq__(self, other):
        return self.qc_assigned_tasks == other.qc_assigned_tasks
    
    def __hash__(self):
        return hash(str(self.qc_assigned_tasks))
    
    def __str__(self):
        result = []
        for qc, tasks in enumerate(self.qc_assigned_tasks, 1):
            result.append(f'QC{qc}: ' + ' -> '.join([str(task) for task in tasks]))

        return "\n".join(result)
    
    def assigned_tasks(self):
        return sum(self.qc_assigned_tasks, [])
    
    def clone(self):
        cloned_state = QCState(len(self.qc_assigned_tasks), self.lpModel.copy())
        cloned_state.qc_assigned_tasks = [tasks.copy() for tasks in self.qc_assigned_tasks]
        cloned_state.qc_completion_time = self.qc_completion_time.copy()
        return cloned_state
    
    def objective(self):
        return max(self.qc_completion_time)

class QCScheduling(SearchProblem):
    M = 9999

    def __init__(self, num_tasks, num_qcs, task_durations, task_locations, qc_locations, non_simultaneous_tasks = {}, precedence_constrained_tasks = {}):
        self.num_tasks = num_tasks
        self.num_qcs = num_qcs
        self.task_durations = task_durations
        self.task_locations = task_locations
        self.qc_locations = qc_locations
        self.non_simultaneous_tasks = non_simultaneous_tasks
        self.precedence_constrained_tasks = precedence_constrained_tasks
    
    def getStartState(self):
        return QCState(self.num_qcs, self.initModel())
    
    def isGoalState(self, state):
        return len(state.assigned_tasks()) == self.num_tasks
    
    def expand(self, state):
        for action in self.getActions(state):
            next_state = self.getNextState(state, action)
            yield (next_state, action, self.getActionCost(state, action, next_state))
    
    def getActions(self, state):
        remaining_tasks = set(range(self.num_tasks)).difference(state.assigned_tasks())
        return [(qc, task) for qc in range(self.num_qcs) for task in remaining_tasks]

    def getActionCost(self, state, action, next_state):
        return 1

    def getNextState(self, state, action):
        qc, task = action
        next_state = state.clone()
        next_state.qc_assigned_tasks[qc].append(task)
        next_state.qc_completion_time[qc] += self.task_durations[task]

        # Add more constraints
        # if len(next_state.qc_assigned_tasks[qc]) > 1:
        #     task_i, task_j = next_state.qc_assigned_tasks[qc][-2:]
        #     next_state.lpModel += self.Xijk[task_i + 1][task_j + 1][qc] == 1 # (4)
        # else:
        #     next_state.lpModel += self.Xijk[0][task + 1][qc] == 1 # (3)
        # next_state.lpModel += self.Yk[qc] >= next_state.qc_completion_time[qc]
        
        return next_state
    
    def initModel(self):
        TASKS = range(self.num_tasks)
        QCS = range(self.num_qcs)

        # Define variables
        self.Xijk = LpVariable.dicts("X", (range(self.num_tasks + 1), range(1, self.num_tasks + 2), QCS), cat="Binary")
        self.Zij = LpVariable.dicts("Z", (TASKS, TASKS), cat="Binary")
        self.Yk = LpVariable.dicts("Y", QCS, lowBound=0)
        self.Di = LpVariable.dicts("D", TASKS, lowBound=0)
        self.C = LpVariable("C", lowBound=0)
        
        # Create model
        prob = LpProblem("QCScheduling", LpMinimize)

        # Objective function (1)
        prob += self.C, "Minimize_makespan"

        # (2)
        init_low_bound = sum(self.task_durations) * 1.0 / self.num_qcs
        prob += self.C >= init_low_bound
        for k in QCS:
            prob += self.C >= self.Yk[k]

        # (3) & (4)
        for k in QCS:
            prob += lpSum([self.Xijk[0][j + 1][k] for j in TASKS]) == 1
            prob += lpSum([self.Xijk[i + 1][self.num_tasks + 1][k] for i in TASKS]) == 1

        # (5)
        for j in TASKS:
            prob += lpSum([self.Xijk[i + 1][j + 1][k] for i in TASKS for k in QCS]) == 1

        # (6)
        for k in QCS:
            for i in TASKS:
                sum_ij = lpSum([self.Xijk[i + 1][j + 1][k] for j in TASKS])
                sum_ji = lpSum([self.Xijk[j + 1][i + 1][k] for j in TASKS])
                prob += sum_ij == sum_ji

        # (7)
        for k in QCS:
            for i in TASKS:
                for j in TASKS:
                    if i == j: continue
                    prob += self.Di[i] + self.task_durations[j] - self.Di[j] <= self.M * (1 - self.Xijk[i + 1][j + 1][k])

        # (8)
        for i, j in self.precedence_constrained_tasks:
            prob += self.Di[i] + self.task_durations[j] <= self.Di[j]

        # (9)
        for i in TASKS:
            for j in TASKS:
                if i == j: continue
                prob += self.Di[i] - self.Di[j] + self.task_durations[j] <= self.M * (1 - self.Zij[i][j])

        # (10)
        for i, j in self.non_simultaneous_tasks:
            prob += self.Zij[i][j] + self.Zij[j][i] == 1
            
        # (12)
        for k in QCS:
            for j in TASKS:
                prob += self.Di[j] - self.Yk[k] <= self.M * (1 - self.Xijk[j + 1][self.num_tasks + 1][k])

        return prob
    
    def addObjectiveUpBound(self, state, upBound):
        state.lpModel += self.C <= upBound
    
    def getTaskCompletionTime(self, state):
        result = {}
        for tasks in state.qc_assigned_tasks:
            completed_time = 0
            for task in tasks:
                if task in result:
                    raise Exception('Something wrong')
                    
                completed_time += self.task_durations[task]
                result[task] = completed_time
        return result
    
    def computeLowBound(self, state):
        remaining_tasks = set(range(self.num_tasks)).difference(state.assigned_tasks())
        sum_ck = sum(state.qc_completion_time)
        sum_pi = sum([self.task_durations[task] for task in remaining_tasks])
        bm = (sum_ck + sum_pi) * 1.0 / self.num_qcs
        return max(max(state.qc_completion_time), bm)
