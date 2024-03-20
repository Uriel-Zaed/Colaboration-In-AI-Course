
from mapf import MapfSolution, Path
from constraint_tree_node import ConstraintTreeNode, Conflict


class Message:
    message_count = 0

    def __init__(self, from_agent: int, to_agent: int):
        Message.message_count += 1
        self.from_agent = from_agent
        self.to_agent = to_agent


class PathForAgentMessage(Message):
    """ A message sent in the initial state to notify other agents of the initial path"""

    def __init__(self, from_agent: int, to_agent: int, path_for_agent: Path):
        super().__init__(from_agent=from_agent, to_agent=to_agent)
        self.path_for_agent = path_for_agent


class DeclareSolutionMessage(Message):
    """ This means the best solution seens so far might be optimal  """

    def __init__(self, from_agent: int, to_agent: int, incumbent: MapfSolution):
        super().__init__(from_agent=from_agent, to_agent=to_agent)
        self.incumbent = incumbent


class DeclareConflictMessage(Message):
    """ This means the best solution seens so far might be optimal  """

    def __init__(self, from_agent: int, to_agent: int, ct_node: ConstraintTreeNode, conflict: Conflict):
        super().__init__(from_agent=from_agent, to_agent=to_agent)
        self.ct_node = ct_node
        self.conflict = conflict


class DeclareOthersConflictMessage(Message):
    """ This means the best solution seens so far might be optimal  """

    def __init__(self, from_agent: int, to_agent: int, ct_node: ConstraintTreeNode):
        super().__init__(from_agent=from_agent, to_agent=to_agent)
        self.ct_node = ct_node



