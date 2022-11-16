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
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """

    # LIFO stack
    frontier = util.Stack()
    # list to keep track of states that have been visited
    visited = []

    start = problem.getStartState()
    # stack will hold state along with the actions to get to that state
    frontier.push((start, []))
    visited.append(start)

    while True:
        # if stack is empty return failure
        if frontier.isEmpty():
            return False
        
        # remove node from stack, retrieve its state and actions
        current = frontier.pop()
        currentState = current[0]
        currentActions = current[1]
        visited.append(currentState)

        if problem.isGoalState(currentState):
            return currentActions
        else:
            # expand current node by getting successors and adding them to the stack
            adj = (problem.getSuccessors(currentState))
            for state, action, cost in adj:
                if state not in visited:
                    # currentActions is list of previous actions
                    # add action to get from current state to this successive state
                    currentPath = currentActions + [action]
                    frontier.push((state, currentPath))
               

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    # similar code to dfs algorithm

    # FIFO queue
    frontier = util.Queue()
    visited = []

    start = problem.getStartState()
    frontier.push((start, []))
    visited.append(start)

    while True:
        if frontier.isEmpty():
            return False
        
        current = frontier.pop()
        currentState = current[0]
        currentActions = current[1]
        visited.append(currentState)

        if problem.isGoalState(currentState):
            return currentActions
        else:
            adj = (problem.getSuccessors(currentState))
            for state, action, cost in adj:
                if state not in visited:
                    currentPath = currentActions + [action]
                    frontier.push((state, currentPath))
                    # unlike dfs, once node is expanded it's visited
                    visited.append(state)


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    
    # priority queue ordered by g-values (cost)
    frontier = util.PriorityQueue()
    # visited is a dictionary to hold the cost it takes to get to different states
    visited = {}

    start = problem.getStartState()
    frontier.push((start, [], 0), 0)

    while True:
        if frontier.isEmpty():
            return False
        
        # remove node from stack, retrieve its state, actions, accumulated cost
        current = frontier.pop()
        currentState = current[0]
        currentActions = current[1]
        currentG = current[2]
        # add accumulated cost to the current state to visited dictionary
        visited[currentState] = currentG

        if problem.isGoalState(currentState):
            return currentActions
        else:
            adj = (problem.getSuccessors(currentState))
            for state, action, cost in adj:
                # if state has been visited but the cost to get to the state is less than
                # previously recorded cost, add new path and cost to priority queue
                if state not in visited or (currentG + cost) < visited[state]:
                    currentPath = currentActions + [action]
                    currentCost = currentG + cost
                    frontier.push((state, currentPath, currentCost), currentCost)
                    visited[state] = currentCost
                    


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    
    # similar code to UCS algorithm

    frontier = util.PriorityQueue()
    visited = {}

    start = problem.getStartState()
    # heuristic used to guess cost
    frontier.push((start, [], 0), heuristic(start, problem))

    while True:
        if frontier.isEmpty():
            return False
        
        current = frontier.pop()
        currentState = current[0]
        currentActions = current[1]
        currentG = current[2]
        visited[currentState] = currentG

        if problem.isGoalState(currentState):
            return currentActions
        else:
            adj = (problem.getSuccessors(currentState))
            for state, action, cost in adj:
                if state not in visited or (currentG + cost) < visited[state]:
                    currentPath = currentActions + [action]
                    currentCost = currentG + cost
                    # priority based on f(x), cost of path from root to x, then x to goal
                    # cost of path from root to x is currentCost (accumulated path cost) + heuristic to guess
                    # cost from current state to the goal
                    frontier.push((state, currentPath, currentCost), (currentCost + heuristic(state, problem)))
                    visited[state] = currentCost

   

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
