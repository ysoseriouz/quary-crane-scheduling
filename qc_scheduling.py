from pulp import *
from search import SearchProblem
import csv
import os


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
            result.append(f'QC{qc}: ' + ' -> '.join([str(task + 1) for task in tasks]))

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
    
    def result(self, action, qcs):
        qc, task = action
        next_state = self.clone()
        next_state.qc_assigned_tasks[qc].append(task)
        next_state.qc_completion_time[qc] += qcs.task_durations[task]

        # Set decision variables
        if len(next_state.qc_assigned_tasks[qc]) > 1:
            task_i, task_j = next_state.qc_assigned_tasks[qc][-2:]
            next_state.addConstraint4(task_i, task_j, qc, qcs)
        else:
            next_state.addConstraint3(task, qc, qcs)
        next_state.addConstraintYk(qc, next_state.qc_completion_time[qc], qcs)
        next_state.addConstraintDi(task, next_state.qc_completion_time[qc], qcs)
        return next_state

class QCScheduling(SearchProblem):
    M = 9999
    ALPHA1 = 1
    ALPHA2 = 0.01

    def __init__(self, num_tasks, num_qcs, task_durations, task_locations, qc_locations, non_simultaneous_tasks = {}, precedence_constrained_tasks = {}):
        self.num_tasks = num_tasks
        self.num_qcs = num_qcs
        self.task_durations = task_durations
        self.task_locations = task_locations
        self.qc_locations = qc_locations
        self.non_simultaneous_tasks = non_simultaneous_tasks
        self.precedence_constrained_tasks = precedence_constrained_tasks
        self.num_ship_bays = max(task_locations)
    
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
        valid_actions = []
        for qc in range(self.num_qcs):
            for task in remaining_tasks:
                if abs(self.qc_locations[qc] - self.task_locations[task]) < (self.num_ship_bays * 1.0 / self.num_qcs):
                    valid_actions.append((qc, task))
        return valid_actions

    def getActionCost(self, state, action, next_state):
        return 1

    def getNextState(self, state, action):
        return state.result(action, self)
    
    def initModel(self):
        TASKS = range(self.num_tasks)
        QCS = range(self.num_qcs)

        # Define variables
        self.Xijk = LpVariable.dicts("X", (range(self.num_tasks + 1), range(1, self.num_tasks + 2), QCS), cat="Binary")
        self.Zij = LpVariable.dicts("Z", (TASKS, TASKS), cat="Binary")
        self.Yk = LpVariable.dicts("Y", QCS, lowBound=0)
        self.Di = LpVariable.dicts("D", TASKS, lowBound=0)
        self.C = LpVariable("C")
        
        # Create model
        prob = LpProblem("QCScheduling", LpMinimize)

        # Objective function (1)
        prob += self.ALPHA1 * self.C + self.ALPHA2 * lpSum([self.Yk[k] for k in QCS]), "Max_Makespan"

        # (2)
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
        # for k in QCS:
        #     for i in TASKS:
        #         for j in TASKS:
        #             if i == j: continue
        #             prob += self.Di[i] + self.task_durations[j] - self.Di[j] <= self.M * (1 - self.Xijk[i + 1][j + 1][k])

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
            prob += self.Zij[i - 1][j - 1] + self.Zij[j - 1][i - 1] == 1

        # (11)
        for k in QCS:
            for i in TASKS:
                for j in TASKS:
                    if self.task_locations[i] < self.task_locations[j]:
                        sum_Xujv = lpSum([self.Xijk[u + 1][j + 1][v] for u in TASKS for v in QCS])
                        sum_Xuiv = lpSum([self.Xijk[u + 1][i + 1][v] for u in TASKS for v in QCS])
                        prob += sum_Xujv - sum_Xuiv <= self.M * (self.Zij[i][j] + self.Zij[j][i])
            
        # (12)
        for k in QCS:
            for j in TASKS:
                prob += self.Di[j] - self.Yk[k] <= self.M * (1 - self.Xijk[j + 1][self.num_tasks + 1][k])

        prob.writeLP("QCScheduling.lp")
        return prob
    
    def inspect1(self, prob, verbose = True, onlyFailed = False):
        TASKS = range(self.num_tasks)
        QCS = range(self.num_qcs)
        final_status = True

        if verbose:
            print('(1)')
            print(f"Objective: {value(prob.objective)}")
            print('(2) C >= Yk[k]')

        C = value(self.C)
        for k in QCS:
            yk = value(self.Yk[k])
            status = C >= yk
            final_status &= status
            if verbose and (not onlyFailed or not status):
                print(f'QC{k + 1}: {C} >= {yk} ({status})')
        
        if verbose:
            print('(3) sum_X0jk')
        for k in QCS:
            values = [value(self.Xijk[0][j + 1][k]) for j in TASKS]
            expr = ' + '.join([str(v) for v in values])
            status = sum(values) == 1
            final_status &= status
            if verbose and (not onlyFailed or not status):
                print(f'QC{k + 1}: {expr} = 1 ({status})')
            
        if verbose:
            print('(4) sum_XiTk')
        for k in QCS:
            values = [value(self.Xijk[i + 1][self.num_tasks + 1][k]) for i in TASKS]
            expr = ' + '.join([str(v) for v in values])
            status = sum(values) == 1
            final_status &= status
            if verbose and (not onlyFailed or not status):
                print(f'QC{k + 1}: {expr} = 1 ({status})')

        if verbose:
            print('(5) sum_Xijk')
        for j in TASKS:
            values = [value(self.Xijk[i + 1][j + 1][k]) for i in TASKS for k in QCS]
            expr = ' + '.join([str(v) for v in values])
            status = sum(values) == 1
            final_status &= status
            if verbose and (not onlyFailed or not status):
                print(f'Task{j + 1}: {expr} = 1 ({status})')

        if verbose:
            print('(6) sum_Xijk - sum_Xjik = 0')
        for k in QCS:
            for i in TASKS:
                sum_ij = [value(self.Xijk[i + 1][j + 1][k]) for j in TASKS]
                sum_ji = [value(self.Xijk[j + 1][i + 1][k]) for j in TASKS]
                expr1 = ' + '.join([str(v) for v in sum_ij])
                expr2 = ' + '.join([str(v) for v in sum_ji])
                status = sum(sum_ij) == sum(sum_ji)
                final_status &= status
                if verbose and (not onlyFailed or not status):
                    print(f'QC{k + 1}, Task{i + 1}: {expr1} = {expr2} ({status})')
        
        if verbose:
            print('(7) Di + Pj - Dj <= M(1 - Xijk)')
        for k in QCS:
            for i in TASKS:
                for j in TASKS:
                    if i == j: continue
                    lv = value(self.Di[i]) + self.task_durations[j] - value(self.Di[j])
                    rv = self.M * (1 - value(self.Xijk[i + 1][j + 1][k]))
                    status = lv <= rv
                    final_status &= status
                    if verbose and (not onlyFailed or not status):
                        print(f'QC{k + 1}, Task{i + 1}, Task{j + 1}: {lv} <= {rv} ({status})')
        
        if verbose:
            print('(8) Di + Pj <= Dj')
        for i, j in self.precedence_constrained_tasks:
            di = value(self.Di[i])
            dj = value(self.Dj[i])
            pj = self.task_durations[j]
            status = di + pj <= dj
            final_status &= status
            if verbose and (not onlyFailed or not status):
                print(f'Task{i + 1}, Task{j + 1}: {lv} <= {rv} ({status})')
        
        if verbose:
            print('(9) Di - Dj + Pj <= M(1 - Zij)')
        for i in TASKS:
            for j in TASKS:
                if i == j: continue
                di = value(self.Di[i])
                dj = value(self.Di[j])
                pj = self.task_durations[j]
                zij = value(self.Zij[i][j])
                status = di - dj + pj <= self.M * (1 - zij)
                final_status &= status
                if verbose and (not onlyFailed or not status):
                    print(f'Task{i + 1}, Task{j + 1}: {di} - {dj} + {pj} <= {self.M} * (1 - {zij}) ({status})')

        if verbose:
            print('(10) Zij + Zji = 1')
        for i, j in self.non_simultaneous_tasks:
            zij = value(self.Zij[i - 1][j - 1])
            zji = value(self.Zij[j - 1][i - 1])
            status = zij + zji == 1
            final_status &= status
            if verbose and (not onlyFailed or not status):
                print(f'Task{i}, Task{j}: {zij} + {zji} = 1 ({status})')
        
        if verbose:
            print('(11) sum_Xujv - sum_Xuiv <= M(Zij + Zji)')
        for k in QCS:
            for i in TASKS:
                for j in TASKS:
                    if self.task_locations[i] < self.task_locations[j]:
                        sum_Xujv = [value(self.Xijk[u + 1][j + 1][v]) for u in TASKS for v in QCS]
                        sum_Xuiv = [value(self.Xijk[u + 1][i + 1][v]) for u in TASKS for v in QCS]
                        expr1 = ' + '.join([str(v) for v in sum_Xujv])
                        expr2 = ' + '.join([str(v) for v in sum_Xuiv])
                        zij = value(self.Zij[i][j])
                        zji = value(self.Zij[j][i])
                        status = sum(sum_Xujv) - sum(sum_Xuiv) <= self.M * (zij + zji)
                        final_status &= status
                        if verbose and (not onlyFailed or not status):
                            print(f'QC{k + 1}, Task{i + 1}, Task{j + 1}: ({expr1}) - ({expr2}) <= {self.M} * ({zij} + {zji}) ({status})')
        
        if verbose:
            print('(12) Dj - Yk <= M(1 - XjTk)')
        for k in QCS:
            for j in TASKS:
                dj = value(self.Di[j])
                yk = value(self.Yk[k])
                xjTk = value(self.Xijk[j + 1][self.num_tasks + 1][k])
                status = dj - yk <= self.M * (1 - xjTk)
                final_status &= status
                if verbose and (not onlyFailed or not status):
                    print(f'QC{k + 1}, Task{j + 1}: {dj} - {yk} <= {self.M} * (1 - {xjTk}) ({status})')
        
        if verbose:
            print(f'Solution ({LpStatus[prob.status]}) - Constraints ({final_status})')
        return final_status

    def inspect2(self, prob, verbose=True, onlyFailed=False):
        soln_dict = {i.name: i.varValue for i in prob.variables()}
        final_status = True
        for c in prob.constraints.values():
            c_dict = c.toDict()
            satisfied = False
            
            LHS = sum([soln_dict[i['name']]*i['value'] for i in c_dict['coefficients']])
            LHS = LHS + c_dict['constant']
            
            if c_dict['sense'] == 0:
                satisfied = (LHS == 0)
        
            if c_dict['sense'] == -1:
                satisfied = (LHS <= 0)
            
            if c_dict['sense'] == 1:
                satisfied = (LHS >= 0)
        
            if verbose and (not onlyFailed or not satisfied):
                print(c, f'({satisfied})')
            final_status &= satisfied

        if verbose:
            print(f'Solution ({LpStatus[prob.status]}) - Constraints ({final_status})')
        return final_status
    
    def export(self, prob, filename, dirname = None):
        if dirname is not None and not os.path.exists(dirname):
            os.makedirs(dirname)
        full_path = f'{filename}.csv' if dirname is None else f'{dirname}/{filename}.csv'
        with open(full_path, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['Status', LpStatus[prob.status]])
            writer.writerow(['Objective', value(prob.objective)])
            for v in prob.variables():
                writer.writerow([v.name, value(v)])    

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
