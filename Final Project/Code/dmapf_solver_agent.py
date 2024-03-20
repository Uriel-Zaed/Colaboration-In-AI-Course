from A_star_algorithm import AStarAlgorithm
from mapf import *
from messages import Message, DeclareConflictMessage, DeclareSolutionMessage, DeclareOthersConflictMessage, \
    PathForAgentMessage
from constraint_tree_node import ConstraintTreeNode, Constraint, Conflict

import copy


class DMapfSolverAgent:
    # Agent states
    INIT_STATE = 0
    IN_PROGRESS_STATE = 1
    DONE_STATE = 2

    def __init__(self, agent_id: int):
        self.agent_id = agent_id  # ID of this agent
        self.mapf_problem = None  # The MAPF problem we want to solve
        self.message_queue = []  # message queue
        self.incumbent_solution = None  # Best solution found so far
        self.state = None
        self._root_solution = None
        self._open_list = None  # Open list for the CBS high level search (on the constraint tree)

    def receive_message(self, message: Message):
        self.message_queue.append(message)

    def setup(self, mapf_problem: MapfProblem):
        self.mapf_problem = mapf_problem
        self.message_queue.clear()  # message queue
        self.incumbent_solution = None  # Best solution found so far
        self.state = DMapfSolverAgent.INIT_STATE
        self._root_solution = MapfSolution()
        self._open_list = list()  # TODO: Shoudl be priority list based on the cost of the solution in each node

        # Find initial path for this agent and send it to all other agents
        my_root_path = self._find_shortest_path(constraints=[])
        self._root_solution[self.agent_id] = my_root_path
        outgoing_messages = []
        for agent in mapf_problem.agents:
            if agent != self.agent_id:
                outgoing_messages.append(PathForAgentMessage(from_agent=self.agent_id,
                                                             to_agent=agent,
                                                             path_for_agent=my_root_path))
        return outgoing_messages

    def _handle_initial_state(self):
        backlog_messages = []  # Message we want to handle later (all the non-init related messages)
        while len(self.message_queue) > 0:
            message = self.message_queue.pop()
            if isinstance(message, PathForAgentMessage):
                self._root_solution[message.from_agent] = message.path_for_agent
            else:
                backlog_messages.append(message)
        self.message_queue.extend(backlog_messages)  # Return the back load messages

        # If received paths for all agents, construct the root CT node
        if len(self._root_solution.keys()) == len(self.mapf_problem.agents):
            if None not in self._root_solution.values():
                root_ct_node = ConstraintTreeNode(self._root_solution, constraints=[])
                self._add_to_open_list(root_ct_node)
            self.state = DMapfSolverAgent.IN_PROGRESS_STATE

    def _handle_message(self, message: Message):
        assert (type(message) != PathForAgentMessage)  # This messages should only be used in the initial state

        if isinstance(message, DeclareSolutionMessage):
            if self.incumbent_solution is None or self.incumbent_solution.cost() >= message.incumbent.cost():
                self.incumbent_solution = message.incumbent
        if isinstance(message, DeclareConflictMessage):
            self.state = DMapfSolverAgent.IN_PROGRESS_STATE
            new_ct_node = self._generate_ct_node(message.ct_node, message.conflict)
            if new_ct_node is not None:
                self._add_to_open_list(new_ct_node)
        if isinstance(message, DeclareOthersConflictMessage):
            conflicts = self._find_conflicts_with_agent(message.ct_node)
            if len(conflicts) != 0:
                self._add_to_open_list(message.ct_node)

    def _declare_incumbent(self):
        """ Create set of messages that declare a new incumbent solution"""
        messages = []
        for agent in self.mapf_problem.agents:
            if agent != self.agent_id:
                messages.append(DeclareSolutionMessage(self.agent_id, agent, self.incumbent_solution))
        return messages

    def _declare_other_conflicts(self, ct_node):
        """ Create set of messages that declare a new incumbent solution"""
        messages = []
        for agent in self.mapf_problem.agents:
            if agent != self.agent_id:
                messages.append(DeclareOthersConflictMessage(from_agent=self.agent_id, to_agent=agent, ct_node=ct_node))
        return messages

    def act(self):
        # *** Handle messages ***
        # If we're still in the initial state, only process the PathForAgent messages to construct the root node of
        # the constraint tree
        if self.state == DMapfSolverAgent.INIT_STATE:
            self._handle_initial_state()
            if len(self._root_solution) < len(self.mapf_problem.agents):
                return []  # Do not start to act before we constructed the initial solution

        while len(self.message_queue) > 0:
            msg = self.message_queue.pop()
            self._handle_message(msg)

        # *** Act: expand agent's own OPEN

        # If OPEN is empty, we might be done. Declare our current best solution (==incumbent)
        if len(self._open_list) == 0:
            self.state = DMapfSolverAgent.DONE_STATE
            if self.incumbent_solution is not None:
                return self._declare_incumbent()
            else:
                return []

        # Else, choose a conflict with this agent to resolve. If non exists, do nothing
        cur_node = self._open_list.pop(0)
        if cur_node.solution.is_valid():
            if self.incumbent_solution is None or self.incumbent_solution.cost() > cur_node.solution.cost():
                self.incumbent_solution = cur_node.solution
                return self._declare_incumbent()

        if self.incumbent_solution is not None and self.incumbent_solution.cost() < cur_node.solution.cost():
            self._open_list.clear()
            return self._declare_incumbent()

        conflicts = self._find_conflicts_with_agent(cur_node)
        if len(conflicts) == 0:  # was valid solution
            if not cur_node.solution.is_valid() and self.incumbent_solution is None:
                return self._declare_other_conflicts(cur_node)
            return []

        else:
            conflict = self._choose_conflict(conflicts)
            # TODO Implement: create a CT node with a constraint for this agent (self.agent_id). Add that node to
            #  this open list

            new_ct_node = self._generate_ct_node(cur_node, conflict)
            # TODO: write func _generate_ct_node
            if new_ct_node is not None:
                self._add_to_open_list(new_ct_node)
            # TODO Implement: send a message to the other agent in this conflict, that will generate CT node that
            #  constrain and replan that agent The resulting CT agent will be added to the open list of that agent
            #  when it handles the message
            message = DeclareConflictMessage(from_agent=self.agent_id,
                                             to_agent=conflict.agent2,
                                             ct_node=cur_node,
                                             conflict=conflict)
            # print(cur_node.solution) print( f"conflict: location {message.conflict.location} from_agent {
            # message.from_agent} to_agent {message.to_agent}")
            return [message]

    def _add_to_open_list(self, new_node):
        if new_node in self._open_list:
            return True
        if len(self._open_list) == 0:
            self._open_list.append(new_node)
            return True
        for cur_node in self._open_list:
            if cur_node > new_node:
                cur_index = self._open_list.index(cur_node)
                self._open_list.insert(cur_index, new_node)
                return True
        self._open_list.append(new_node)
        return True

    def _find_conflicts_with_agent(self, ct_node: ConstraintTreeNode):
        """ Return conflicts with this (=self.agent_id) agent  """
        # TODO Implement me. IMPORTANT: Ensure in the created conflicts conflict.agent1 is always self.agent_id
        confilcts = []
        for other_agent in ct_node.solution.keys():
            if other_agent != self.agent_id:
                pathLength = min(len(ct_node.solution[other_agent]), len(ct_node.solution[self.agent_id]))
                for i in range(pathLength):
                    if ct_node.solution[other_agent][i] == ct_node.solution[self.agent_id][i]:
                        confilcts.append(Conflict(ct_node.solution[other_agent][i], i, agent1=self.agent_id,
                                                  agent2=other_agent))
        return confilcts

    def _generate_ct_node(self, ct_node: ConstraintTreeNode, conflict: Conflict):
        """ Create a single CT node by adding a constraint to our agent  and replanning for it """
        new_constraint = Constraint(conflict.location, conflict.time_step, self.agent_id)
        new_constraints = copy.deepcopy(ct_node.constraints)
        new_constraints.append(new_constraint)
        new_path = self._find_shortest_path(new_constraints)
        if new_path is None:
            return None
        new_mapf_sol = copy.deepcopy(ct_node.solution)
        new_mapf_sol[self.agent_id] = new_path
        return ConstraintTreeNode(new_mapf_sol, new_constraints)

    def _choose_conflict(self, conflicts: list):
        # TODO Any way to choose is Ok for now
        return conflicts[0]

    def is_done(self):
        """ Returns true if the agent has an incumbent solution and no potentially better solutions to examine """
        return self.state == DMapfSolverAgent.DONE_STATE

    def _find_shortest_path(self, constraints):
        # Find the shortest path for self.agent_id from its start to its goals subject to the given set of constraints
        # (= standard low-level search in CBS)
        agent_const = []
        for const in constraints:
            if const.agent_id == self.agent_id:
                agent_const.append(const)

        starting_state = copy.deepcopy(self.mapf_problem.terrain)
        init_location = self.mapf_problem.sources[self.agent_id]
        starting_state[init_location[0]][init_location[1]] = "*"

        goal_state = copy.deepcopy(self.mapf_problem.terrain)
        goal_location = self.mapf_problem.targets[self.agent_id]
        goal_state[goal_location[0]][goal_location[1]] = "*"

        node_path = AStarAlgorithm(starting_state, goal_state, constraints, self.agent_id, self.mapf_problem)

        if node_path is None:
            return None
        path = Path()
        while node_path.parent is not None:
            path.append(node_path.get_cur_pos())
            node_path = node_path.parent
            if node_path.parent is None:
                path.append(node_path.get_cur_pos())
                path.reverse()
        return path
