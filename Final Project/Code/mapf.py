
class MapfProblem:
    def __init__(self, sources: list, targets: list, terrain, agents: list):
        self.sources = sources  # Initial state
        self.targets = targets
        self.agents = agents
        self.terrain = terrain
        self.heuristic_maps = {}


class Path(list):
    # This is a path for a single agent.
    # Should be a sequence of steps or something like that
    def __init__(self):
        super().__init__(self)


class MapfSolution(dict):
    """ This is a MAPF solution, maybe with conflicts.
    Should be a mapping between agent id to a path """
    def __init__(self):
        super().__init__(self)

    def is_valid(self):
        for a1 in self.keys():
            for a2 in self.keys():
                if a1 != a2:
                    pathLength = min(len(self[a1]), len(self[a2]))
                    for i in range(pathLength):
                        if self[a1][i] == self[a2][i]:
                            return False
        return True

    def __str__(self):
        st = ""
        for i in self.keys():
            st += f"{i}: {self[i]} \n"
        return st

    def cost(self):
        sumCost = 0
        for a in self.keys():
            sumCost += len(self[a])
        return sumCost
