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
    # python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=dfs
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
    #python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=bfs
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    from util import Queue

    """
        initialiseer je set (om bij te houden of je die node al hebt doorzocht,
        ook je Queue wordt eerst geïnitialiseerd en dan wordt de start node hierop
        gepushed.
    """
    bfsQueue = Queue()  # Voor de Breadth first search gaan we een queue gebruiken ipv een stack
    bfsVisited = []
    """
    Als onze start state al de goal state is gaan we een lege queue terug sturen. (Want initele positie is ook de eindpositie)
    """
    if problem.isGoalState(problem.getStartState()):
        return []

    """
    Is dit niet zo zoek verder en push de start op queue
    """
    bfsQueue.push((problem.getStartState(),[]))

    while bfsQueue:

        #Als er niets meer in de queue zit gaan we uit de loop gaan, dit heb ik aangepast door while bfsQueue te plaatsen dus is overbodig nu
        # if bfsQueue.isEmpty():
        #     return []

        node,path = bfsQueue.pop() #Neem de huidige node en het pad
        bfsVisited.append(node)
        print(node, end="")

        if problem.isGoalState(node): #Als je uitkomt bij je goal, dan return je de lijst van acties die nodig waren (=het path)
            return path

        nextElement = problem.getSuccessors(node) #Error handling en leesbaarheid

        if nextElement:
            for successor in nextElement: # Haal de volgende state en actie uit de neighbour
                if successor[0] not in bfsVisited and successor[0] not in (state[0] for state in bfsQueue.list): #

                    # Even wat uitleg: Neem de eerstvolgende successor en we kijken of deze nog niet bezocht is.
                    # Ook ki9jken ze of hij nog niet in 1 van de states staat van de Queue want dan moeten we dat niet meer bekijken
                    newPath = path + [successor[1]]  # Calculate new path
                    bfsQueue.push((successor[0], newPath))


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue

    ucsVisited = []
    ucsQueue = PriorityQueue() #De queue bestaat uit: (x,y), [path] en priority

    if problem.isGoalState(problem.getStartState()):
        return []

    ucsQueue.push((problem.getStartState(), []),0) #Begin bij het start punt, de cost en prioriteit = 0

    while not ucsQueue.isEmpty():

        node, path = ucsQueue.pop() #Neem de huidige node en de acties die we al hebben genomen
        ucsVisited.append(node)

        if problem.isGoalState(node): #Als je uitkomt bij je goal, dan return je de lijst van acties die nodig waren (=het path)
            return path

        # Bekijk alle nabije mogelijke paden van de huidige positie
        for nextState, action, cost in problem.getSuccessors(node):
            if nextState not in ucsVisited:
                new_path = path + [action]
                new_cost = problem.getCostOfActions(new_path)

                # Als deze state nog niet toegevoegd was of als er een 'cheaper' state gevonden is
                if nextState not in (state[2][0] for state in ucsQueue.heap): #[2] geeft onze het 3e element --> de priority, [0] de coordinaten
                    ucsQueue.push((nextState, new_path), new_cost)
                else:
                    # Als er al een pad bestaat naar de state, kijk of hij goedkoper is.
                    for state in ucsQueue.heap:
                        if state[2][0] == nextState:
                            old_cost = problem.getCostOfActions(state[2][1])#[2] prioriteit, [1] path
                    if new_cost < old_cost:
                        ucsQueue.update((nextState, new_path), new_cost)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    #python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
    """Search the node that has the lowest combined cost and heuristic firt."""
    "*** YOUR CODE HERE ***"
    aStarQueue = util.PriorityQueue()
    actions = []
    state = problem.getStartState()
    aStarQueue.push((state,actions,0),0)
    visited = set()
    while (not problem.isGoalState(state)):
        xy, actions, totalCost = aStarQueue.pop()
        if (problem.isGoalState(xy)):
            return actions
        if (xy not in visited):
            visited.add(xy)
            for successor, action, cost in problem.getSuccessors(xy):
                new_actions = actions + [action]
                aStarQueue.push((successor,new_actions,cost+totalCost),cost+totalCost+heuristic(successor,problem))
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch