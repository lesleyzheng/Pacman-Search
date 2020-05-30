# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
import util


class Node:

    def __init__(self, problem, stateCurrent, cost=1, heuristicCost = 0, nodePrev = None, action=None):
        self.nodeProblem = problem
        self.nodeCurrentState = stateCurrent
        self.nodePrevNode = nodePrev
        self.nodeAction = action
        self.nodeCost = cost
        self.nodeHeuristicCost = heuristicCost

    def nodeGetCurrentState(self):

        return self.nodeCurrentState

    def nodeGetPrevState(self):

        return self.nodePrevNode

    def nodeGetProblem(self):

        return self.nodeProblem

    def nodeGetAction(self):

        return self.nodeAction

    def nodeGetCost(self):

        return self.nodeCost

    def nodeGetHeuristicCost(self):

        return self.nodeHeuristicCost


def Solution(backTraceStartNode):

    currentNode = backTraceStartNode
    actions = []
    costs = []
    actions.append(currentNode.nodeGetAction())
    costs.append(currentNode.nodeGetHeuristicCost() - currentNode.nodeGetCost())

    while currentNode.nodeGetPrevState() != None:

        currentNode = currentNode.nodeGetPrevState()

        if currentNode.nodeGetAction() != None:
            actions.append(currentNode.nodeGetAction())
            costs.append(currentNode.nodeGetHeuristicCost() - currentNode.nodeGetCost())

    actions.reverse()
    costs.reverse()

    return actions


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).
    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state
        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state
        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take
        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.
    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.
    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """

    nodeInit = Node(problem=problem, stateCurrent=problem.getStartState())

    if problem.isGoalState(nodeInit.nodeGetCurrentState()):

        return Solution(nodeInit)

    frontier = util.Stack()
    frontier.push(nodeInit)

    explored = set()

    while frontier.isEmpty() != True:

        currentNode = frontier.pop()
        if problem.isGoalState(currentNode.nodeGetCurrentState()):
            return Solution(currentNode)
        if currentNode.nodeGetCurrentState() not in explored:
            explored.add(currentNode.nodeGetCurrentState())

            for successor in problem.getSuccessors(currentNode.nodeGetCurrentState()):

                childNode = Node(problem=problem, stateCurrent=successor[0], nodePrev=currentNode, action=successor[1])

                if childNode.nodeGetCurrentState() not in explored:

                    frontier.push(childNode)

    return None


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    nodeInit = Node(problem=problem, stateCurrent=problem.getStartState())

    if problem.isGoalState(nodeInit.nodeGetCurrentState()):

        return Solution(nodeInit)

    frontier = util.Queue()
    frontier.push(nodeInit)

    explored = set()

    while frontier.isEmpty() != True:

        currentNode = frontier.pop()
        if problem.isGoalState(currentNode.nodeGetCurrentState()):
            return Solution(currentNode)

        if currentNode.nodeGetCurrentState() not in explored:
            explored.add(currentNode.nodeGetCurrentState())

            for successor in problem.getSuccessors(currentNode.nodeGetCurrentState()):

                childNode = Node(problem=problem, stateCurrent=successor[0], nodePrev=currentNode, action=successor[1])

                if childNode.nodeGetCurrentState() not in explored:
                    frontier.push(childNode)

    return None


def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    nodeInit = Node(problem=problem, stateCurrent=problem.getStartState()) # don't need to set cost bc its the first and only item so far

    if problem.isGoalState(nodeInit.nodeGetCurrentState()):

        return Solution(nodeInit)

    frontier = util.PriorityQueue()
    frontier.push(nodeInit, 0)

    explored = set()

    while frontier.isEmpty() != True:


        currentNode = frontier.pop()
        if problem.isGoalState(currentNode.nodeGetCurrentState()):
            return Solution(currentNode)

        if currentNode.nodeGetCurrentState() not in explored:
            explored.add(currentNode.nodeGetCurrentState())

            for successor in problem.getSuccessors(currentNode.nodeGetCurrentState()):

                childNode = Node(problem=problem, stateCurrent=successor[0], nodePrev=currentNode, action=successor[1], cost=successor[2] + currentNode.nodeGetCost())

                if childNode.nodeGetCurrentState() not in explored:

                    frontier.push(childNode, childNode.nodeGetCost())

    return None


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarCost(gn, position, problem, heuristic=nullHeuristic):
    # gn is the total cost up until position
    # hn is estimated cost from position to goal
    # fn is total estimated cost from start to goal along that path
    hn = heuristic(position, problem)
    fn = hn+gn

    return fn


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    nodeInit = Node(problem=problem,
                    stateCurrent=problem.getStartState())  # don't need to set cost bc its the first and only item so far

    if problem.isGoalState(nodeInit.nodeGetCurrentState()):
        return Solution(nodeInit)

    frontier = util.PriorityQueue()
    frontier.push(nodeInit, 0)

    explored = set()

    while frontier.isEmpty() != True:

        currentNode = frontier.pop()
        if problem.isGoalState(currentNode.nodeGetCurrentState()):
            return Solution(currentNode)
        if currentNode.nodeGetCurrentState() not in explored:
            explored.add(currentNode.nodeGetCurrentState())
            for successor in problem.getSuccessors(currentNode.nodeGetCurrentState()):

                childNode = Node(problem=problem, stateCurrent=successor[0], nodePrev=currentNode, action=successor[1],
                                 cost=
                                 successor[2] + currentNode.nodeGetCost(), ## g(n)

                                 heuristicCost=aStarCost(
                                     gn=(successor[2] + currentNode.nodeGetCost()),
                                                         position=successor[0], problem=problem, heuristic=heuristic)) ##actually is fn
                if childNode.nodeGetCurrentState() not in explored:

                    frontier.push(childNode, childNode.nodeGetHeuristicCost())

    return None


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch