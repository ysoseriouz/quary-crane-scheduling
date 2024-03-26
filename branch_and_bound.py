from time import time
from pulp import *
from search import *
import util


def branch_and_bound_dfs(qcs):
    fringe = util.Stack()
    explored = set()
    start_node = Node(state=qcs.getStartState(), parent=None, action=None)
    fringe.push(start_node)

    objective = float('inf')
    solution = None
    count = 0

    start_time = time()
    while not fringe.isEmpty():
        count += 1
        if count % 100 == 0:
            print('ITERATION %d' % count)
        node = fringe.pop()
        node.state.lpModel.solve(PULP_CBC_CMD(msg=False))

        if objective <= value(node.state.lpModel.objective) or LpStatus[node.state.lpModel.status] != 'Optimal':
            explored.add(node.state)
        elif qcs.isGoalState(node.state):
            explored.add(node.state)
            incumbent_objective = value(node.state.lpModel.objective)
            if objective > incumbent_objective and LpStatus[node.state.lpModel.status] == 'Optimal':
                objective = incumbent_objective
                solution = node
        else:
            # Branching
            feasible_child_nodes = []
            if node.state not in explored:
                explored.add(node.state)
                for child, action, cost in qcs.expand(node.state):
                    if child not in explored:
                        child_node = Node(state=child, parent=node, action=action, 
                                            depth=node.depth + 1, cost=node.cost + cost)
                        feasible_child_nodes.append(child_node)

            # TODO: wtf is this even mean
            delete_dominated_nodes(qcs, feasible_child_nodes, explored)

            # Calculate lower bound and prune sub-tree
            for child_node in feasible_child_nodes:
                if qcs.computeLowBound(child_node.state) >= objective:
                    explored.add(child_node.state)
                else:
                    fringe.push(child_node)

    print(f'Done in {time() - start_time}(s) with {count}(iters)')
    return solution.state if solution is not None else None

def delete_dominated_nodes(qcs, nodes, explored):
    pass
