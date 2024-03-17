import util

class SearchProblem:
    def getStartState(self):
        util.raiseNotDefined()

    def isGoalState(self, state):
        util.raiseNotDefined()

    def expand(self, state):
        util.raiseNotDefined()

    def getActions(self, state):
        util.raiseNotDefined()

    def getActionCost(self, state, action, next_state):
        util.raiseNotDefined()

    def getNextState(self, state, action):
        util.raiseNotDefined()

    def getCostOfActionSequence(self, actions):
        return len(actions)

class Node:
    def __init__(self, state, parent, action, depth=0, cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.depth = depth
        self.cost = cost
    
    def __eq__(self, other):
        return self.state == other.state and self.parent == other.parent \
            and self.action == other.action and self.depth == other.depth \
            and self.cost == other.cost
    
    def __hash__(self):
        return hash(self.state) + hash(self.parent) + hash(self.action) + self.depth + self.cost
    
    def __str__(self):
        return f"{self.parent}->{self.state}"
