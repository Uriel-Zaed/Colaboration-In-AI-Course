
from mapf import MapfSolution


class ConstraintTreeNode:
    """ A node in the constraint tree """

    def __init__(self, solution: MapfSolution, constraints: list):
        self.solution = solution
        self.constraints = constraints

    def __eq__(self, other):
        return self.solution.cost() == other.solution.cost()

    def __gt__(self, other):
        return self.solution.cost() > other.solution.cost()


class Conflict:
    """ A conflict between agents """

    def __init__(self, location, time_step, agent1, agent2):
        self.location = location
        self.time_step = time_step
        self.agent1 = agent1
        self.agent2 = agent2


class Constraint:
    """ A constraint on a single agent """

    def __init__(self, location, time_step, agent_id):
        self.location = location
        self.time_step = time_step
        self.agent_id = agent_id
