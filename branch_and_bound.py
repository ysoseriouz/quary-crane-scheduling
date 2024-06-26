from time import time
from pulp import *
from search import *
import util
import logging

TIME_LIMIT = 10800 # 3 hours

logger = logging.getLogger()

def branch_and_bound_dfs(qcs):
    logger.info('Branch and bound---------------------')
    fringe = util.PriorityQueue()
    explored = set()
    start_node = qcs.getStartState()
    fringe.push(start_node, qcs.computeLowBound(start_node))

    objective = float('inf')
    solution = None
    count = 0

    start_time = time()
    while not fringe.isEmpty():
        count += 1
        if count % 100 == 0:
            logger.info('ITERATION %d' % count)
            if time() - start_time >= TIME_LIMIT:
                logger.info('Time limit exceeded')
                break
        node = fringe.pop()
        node.lpModel.solve(PULP_CBC_CMD(msg=False))

        if objective <= value(node.lpModel.objective) or not node.isFeasible():
            explored.add(node)
        elif qcs.isGoalState(node):
            incumbent_objective = value(node.lpModel.objective)
            if objective > incumbent_objective and node.isFeasible():
                objective = incumbent_objective
                solution = node
                logger.info(f'(ITERATION {count}) New solution found: {node.objective()}')
                logger.info(solution)
        else:
            # Branching
            feasible_child_nodes = []
            if node not in explored:
                explored.add(node)
                for child, _, _ in qcs.expand(node):
                    if child not in explored:
                        feasible_child_nodes.append(child)

                feasible_child_nodes = delete_dominated_nodes(feasible_child_nodes)

                # Calculate lower bound and prune sub-tree
                for child_node in feasible_child_nodes:
                    minimum_lower_bound = qcs.computeLowBound(child_node)
                    if minimum_lower_bound < objective:
                        fringe.push(child_node, minimum_lower_bound)

    run_time = time() - start_time
    logger.info(f'Done in {run_time}(s) with {count}(iters)')
    return solution, run_time

def delete_dominated_nodes(nodes):
    dnodes = []
    for node in nodes:
        found = False
        found_idx = -1
        for idx, dnode in enumerate(dnodes):
            if node.match(dnode):
                found = True
                if node.dominate(dnode):
                    found_idx = idx
            if found: break

        if found_idx >= 0:
            dnodes[found_idx] = node
        elif not found:
            dnodes.append(node)

    return dnodes
