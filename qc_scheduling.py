from itertools import product
from pulp import *
from search import SearchProblem

class QCState:
    def __init__(self, num_qcs, lpModel = None):
        self.qc_assigned_tasks = [[] for _ in range(num_qcs)]
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
        next_state = QCState(len(state.qc_assigned_tasks), state.lpModel.copy())
        next_state.qc_assigned_tasks = [tasks.copy() for tasks in state.qc_assigned_tasks]
        next_state.qc_assigned_tasks[qc].append(task)

        # Add more constraints
        if len(next_state.qc_assigned_tasks[qc]) == 1:
            # Constraint 3
            next_state.lpModel += self.X0jk[task][qc] == 1
            next_state.lpModel += self.Yk[qc] >= self.task_durations[task]
        else:
            # Constraint 5
            task_i, task_j = next_state.qc_assigned_tasks[qc][-2:]
            next_state.lpModel += self.Xijk[task_i][task_j][qc] == 1
            next_state.lpModel += self.Yk[qc] >= self.get_qc_completion_time(next_state)[qc]

        return next_state
    
    def initModel(self):
        TASKS = range(self.num_tasks)
        QCS = range(self.num_qcs)

        # Define variables
        self.Xijk = LpVariable.dicts("X", (TASKS, TASKS, QCS), cat="Binary")
        self.X0jk = LpVariable.dicts("X0", (TASKS, QCS), cat="Binary")
        self.XTjk = LpVariable.dicts("XT", (TASKS, QCS), cat="Binary")
        self.Zij = LpVariable.dicts("Z", (TASKS, TASKS), cat="Binary")

        self.Yk = LpVariable.dicts("Y", QCS, lowBound=0)
        self.Di = LpVariable.dicts("D", TASKS, lowBound=0)
        self.C = LpVariable("C", lowBound=0)
        
        # Create model
        prob = LpProblem("QCScheduling", LpMinimize)

        # Objective function
        prob += self.C, "Minimize_makespan"

        # Constraint 2
        for k in QCS:
            prob += self.Yk[k] <= self.C

        # # Constraint 3 & 4
        for k in QCS:
            prob += lpSum([self.X0jk[j][k] for j in TASKS]) == 1 # (3)
            prob += lpSum([self.XTjk[j][k] for j in TASKS]) == 1 # (4)

        # # Constraint 5
        for j in TASKS:
            sum_Xijk = lpSum([self.Xijk[i][j][k] for i in TASKS for k in QCS])
            sum_X0jk = lpSum([self.X0jk[j][k] for k in QCS])
            prob += sum_Xijk + sum_X0jk == 1

        # Constraint 6
        # for i in TASKS:
        #     for k in QCS:
        #         sum_Xjik = lpSum([self.Xijk[j][i][k] for j in TASKS])
        #         sum_Xijk = lpSum([self.Xijk[i][j][k] for j in TASKS])
        #         prob += sum_Xjik + self.X0jk[i][k] - sum_Xijk - self.XTjk[i][k] == 1

        # Constraint 7
        # for i, j in product(TASKS, TASKS):
        #     for k in QCS:
        #         prob += self.Di[i] + self.task_durations[i] - self.Di[j] <= self.M * (1 - self.Xijk[i][j][k])

        # Constraint 8
        for i, j in self.precedence_constrained_tasks:
            self.Di[i] + self.task_durations[j] <= self.Di[j]

        # # Constraint 9
        # for i, j in product(TASKS, TASKS):
        #     if i == j: continue
        #     for k in QCS:
        #         prob += self.Di[i] - self.Di[j] + self.task_durations[j] <= self.M * (1 - self.Zij[i][j])

        # Constraint 10
        for i, j in self.non_simultaneous_tasks:
            prob += self.Zij[i][j] + self.Zij[j][i] == 1
            
        # Constraint 12
        for j in TASKS:
            for k in QCS:
                prob += self.Di[j] - self.Yk[k] <= self.M * (1 - self.XTjk[j][k])

        # Custom constraint
        for k in QCS:
            prob += self.Yk[k] >= min(self.task_durations)

        return prob
    
    def add_objective_upperbound(self, state, upperbound):
        state.lpModel += self.C <= upperbound
        
    def add_objective_lowerbound(self, state, lowerbound):
        state.lpModel += self.C >= lowerbound
    
    def get_task_completion_time(self, state):
        result = {}
        for tasks in state.qc_assigned_tasks:
            completed_time = 0
            for task in tasks:
                if task in result:
                    raise Exception('Something wrong')
                    
                completed_time += self.task_durations[task]
                result[task] = completed_time
        return result
    
    def get_qc_completion_time(self, state):
        result = {}
        for qc, tasks in enumerate(state.qc_assigned_tasks):
            result[qc] = sum([self.task_durations[task] for task in tasks])
        return result
    
    def lower_bound(self, state):
        remaining_tasks = set(range(self.num_tasks)).difference(state.assigned_tasks())
        sum_remaining_task_duration = sum([self.task_durations[task] for task in remaining_tasks])
        Ck = self.get_qc_completion_time(state)
        sum_qc_completed_time = sum(Ck.values())
        bm = (sum_remaining_task_duration + sum_qc_completed_time) * 1.0 / self.num_qcs
        return max(max(Ck.values()), bm)
