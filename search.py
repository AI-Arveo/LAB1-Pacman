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
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    initialiseer je set (om bij te houden of je die node al hebt doorzocht,
    ook je stack wordt eerst geïnitialiseerd en dan wordt de start node hierop
    gepushed.
    """
    visited = set()
    dfsStack = util.Stack()
    dfsStack.push(problem.getStartState())
    while (not dfsStack.isEmpty()):
        """
        Je popt de eerste op de stack om deze te gaan doorzoeken. Dit is een tuple van de
        positie van de node en van de actie die is ondernomen om daar te geraken. Bij de
        start node is dit bv.: (5,5) hierbij is geen action omdat je ook nog niets hebt 
        moeten doen om hier te geraken. 
        """
        node, actions = dfsStack.pop()
        """
        Wat gebeurt er als de state wel al een keer doorlopen is?
        -> dan doe je verder niets. Maar dan pop je die enkel (zie hierboven)
        De while lus is nog niet gedaan en dan check je de andere nodes op de zelfde layer
        van je tree. Als deze nog niet gezien is ga je hier dieper in. Als deze ook al gezien
        pop je ook deze. Zodat je een layer hoger in je tree gaat tot je een node vind die
        je nog niet hebt bekeken.
        """
        if (node not in visited):
            """
            Als je de state/positie nog niet hebt gezien, dan maak zet je dit in je set
            zodat je vanaf dan weet dat die node wel is doorlopen
            """
            visited.add(node)
            if (problem.isGoalState(node)):
                """
                Als je uitkomt bij je goal, dan geef je de lijst van acties die nodig waren
                zoals bij tinyMazeSearch
                """
                return actions
            for successor in problem.getSuccessors(node):
                """
                Als je niet je goal hebt bereikt, zet je alle successors op de stack.
                Doordat de stack LIFO is wordt steeds de eerste node bekeken
                => depth first, tot je ooit een state tegenkomt die je al eens hebt gezien
                hebt -> dus in visited. Dan begin je ze te poppen en dus terug hoger in je
                tree te gaan. 
                """
                if (successor not in visited):
                    dfsStack.push(successor)
    util.raiseNotDefined()

def breadthFirstSearch(problem):
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
    ucsQueue = PriorityQueue()

    if problem.isGoalState(problem.getStartState()):
        return []
    ucsQueue.push((problem.getStartState(), [], 0),0)
    current_cost = 0

    while ucsQueue:
        node, path, cost = ucsQueue.pop()#Neem de huidige node en de acties die we al hebben genomen
        ucsVisited.append(node)

        if problem.isGoalState(node): #Als je uitkomt bij je goal, dan return je de lijst van acties die nodig waren (=het path)
            return path

        nextElement = problem.getSuccessors(node)

        #state = problem.getCostOfActions(node)
        current_cost += cost
        #Voeg nieuwe states toe an de queue
        if nextElement:
            for successor, cost in nextElement:
                total_cost = 0
                total_cost += cost
                if successor not in ucsVisited or total_cost<ucsVisited[successor][0]:
                    newPath = path + [successor[1]]
                    ucsVisited[successor][0] = (successor[0], newPath,total_cost)

                    cost = problem.getCostOfActions(newPath)

                    ucsQueue.push((successor[0], newPath), cost)

                # elif successor[0] not in ucsVisited or total_cost<current_cost:
                #
                #     newCost = problem.getCostOfActions(path + [successor[1]])
                #
                #     # Als de huidige cost hoger is dan de nieuwe cost gaan we het pad veranderen
                #     if cost > newCost:
                #         newPath = path + [successor[1]]
                #         ucsQueue.update((successor[0], newPath), newCost)


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
