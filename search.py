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
