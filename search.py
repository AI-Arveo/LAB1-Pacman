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
    "*** YOUR CODE HERE ***"
    visited = set()
    dfsStack = util.Stack()
    actions = []
    dfsStack.push((problem.getStartState(),actions))
    state = problem.getStartState()

    """
    Je popt de eerste op de stack om deze te gaan doorzoeken. Dit is een tuple van de
    positie van de node en van de actie die is ondernomen om daar te geraken. Bij de
    start node is dit bv.: (5,5) hierbij is geen action omdat je ook nog niets hebt 
    moeten doen om hier te geraken. 
    """
    while (not problem.isGoalState(state)):
        # pop de stack om je xy te krijgen en je actions (wat een list [] is met alle acties nodig om tot
        # die xy waarde te komen.
        xy, actions = dfsStack.pop()
        # markeer die xy als visited zodat je hier niet nog eens naartoe gaat. En dus een oneindige lus
        # voorkomt
        visited.add(xy)
        # check of die gepopte xy coordinaat de goal is, zoja return de actions die hierbij op de stack staan.
        if (problem.isGoalState(xy)):
            return actions
        # itereer over de successors (kan ook reversed(..)), de cost is hierbij overbodig. Maar dit kan handig zijn
        # voor andere algoritmes met cost
        for successor, action, costs in problem.getSuccessors(xy):
            # als je de successor nog niet bezocht hebt, push je die op de stack. Samen met de actions om tot bij
            # die successor te geraken.
            if (successor not in visited):
                # De actions bereken je door action (van de successor) toe te voegen bij de actions list
                # LET OP: push niet actions.append(..) op de stack. Als je dit doet, dan maak je geen
                # copie en ben je dus met references bezig => als je actions later aanpast => wordt ook
                # de stack aangepast.
                # Werk daarom met een nieuwe list (new_actions) die een copie is met de nieuwe action erbij.
                # dit doe je met de lijn hieronder!
                new_actions = actions + [action]
                dfsStack.push((successor, new_actions))
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

    """
        initialiseer je set (om bij te houden of je die node al hebt doorzocht,
        ook je stack wordt eerst geïnitialiseerd en dan wordt de start node hierop
        gepushed.
        """
    bfsVisited = []
    bfsQueue = util.Queue() #Voor de Breadth first search gaan we een queue gebruiken ipv een stack

    node = problem.getStartState()

    """
    Als ons problem al de goal state is gaan we een lege queue terug sturen. (Want initele positie is ook de eindpositie)
    """
    if problem.isGoalState(node):
        return []

    """
    Is dit niet zo zoek verder en push de start op queue
    """
    bfsQueue.push(node)
    while (not bfsQueue.isEmpty()):
        node,actions = bfsQueue.pop()
        bfsVisited.append(node) #Voeg de node toe aan visited

        if (problem.isGoalState(node)): #Als je uitkomt bij je goal, dan return je de lijst van acties die nodig waren (=de queue)
            return actions
        if problem.getSuccessors(node):
            for successor in problem.getSuccessors(node): # Haal de volgende state en actie uit de neighbour
                if successor[0] not in bfsVisited and successor[0] not in bfsQueue.list:
                    # Even wat uitleg: We kijken of onze succesor nog niet bezocht is en of hij nog niet in de queue staat,

                    nextAction = actions + [successor[1]] # bereken het nieuwe pad
                    bfsQueue.push(successor[0], nextAction)
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
