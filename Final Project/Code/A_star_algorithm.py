
import copy


class AStar_Node:
    def __init__(self, state, path_cost, parent, goal_state, agent_id, mapf_problem):
        self.state = state
        self.path_cost = path_cost
        self.goal_state = goal_state
        self.heuristic_val = self.heuristic_calculation(agent_id, mapf_problem)
        self.parent = parent

    def get_cur_pos(self):
        for i in range(len(self.state)):
            for j in range(len(self.state)):
                if self.state[i][j] == "*":  # current position of agent
                    return [i, j]

    def getChildren(self, agent_id, mapf_problem):  # Get all children/actions and add them to the children array
        children = []
        i = self.get_cur_pos()[0]
        j = self.get_cur_pos()[1]
        if i > 0 and self.state[i - 1][j] == ".":  # Up
            new_state = copy.deepcopy(self.state)
            new_state[i - 1][j] = "*"
            new_state[i][j] = "."
            children.append(AStar_Node(new_state, self.path_cost + 1, self, self.goal_state, agent_id, mapf_problem))
        if i < len(self.state) - 1 and self.state[i + 1][j] == ".":  # down
            new_state = copy.deepcopy(self.state)
            new_state[i + 1][j] = "*"
            new_state[i][j] = "."
            children.append(AStar_Node(new_state, self.path_cost + 1, self, self.goal_state, agent_id, mapf_problem))
        if j < len(self.state[i]) - 1 and self.state[i][j + 1] == ".":  # Right
            new_state = copy.deepcopy(self.state)
            new_state[i][j + 1] = "*"
            new_state[i][j] = "."
            children.append(AStar_Node(new_state, self.path_cost + 1, self, self.goal_state, agent_id, mapf_problem))
        if j > 0 and self.state[i][j - 1] == ".":  # left
            new_state = copy.deepcopy(self.state)
            new_state[i][j - 1] = "*"
            new_state[i][j] = "."
            children.append(AStar_Node(new_state, self.path_cost + 1, self, self.goal_state, agent_id, mapf_problem))
        children.append(AStar_Node(self.state, self.path_cost + 1, self, self.goal_state, agent_id, mapf_problem))
        return children

    def heuristic_calculation(self, agent_id, mapf_problem):
        return mapf_problem.heuristic_maps[agent_id][self.get_cur_pos()[0]][self.get_cur_pos()[1]]


def AStarAlgorithm(starting_state, goal_state, constraints, agent_id, mapf_problem):
    node = AStar_Node(starting_state, 0, None, goal_state, agent_id, mapf_problem)
    frontier = [node]
    explored = []
    explored = list(set(explored))

    i = 0
    while len(frontier) > 0:
        i += 0
        if i > len(starting_state) * len(starting_state) * len(starting_state) * 2:
            return None
        node = frontier.pop(0)

        if node.state == goal_state:
            return node

        explored.append(node)

        children = node.getChildren(agent_id, mapf_problem)

        frontier = add_children(frontier, children, explored, constraints)

    return None


def add_children(frontier, children, explored, constraints):
    for child in children:
        if constrained(child, constraints) or is_node_in_list(frontier, child) or is_node_in_list(explored, child):
            continue
        if len(frontier) == 0:
            frontier.append(child)
            continue
        cur_index = smart_index(frontier, child)
        frontier.insert(cur_index, child)

    return frontier


def smart_index(frontier, child):
    for cur_node in frontier:  # Smart add
        if cur_node.path_cost + cur_node.heuristic_val > child.path_cost + child.heuristic_val:
            return frontier.index(cur_node)
        if cur_node.path_cost + cur_node.heuristic_val == child.path_cost + child.heuristic_val:
            if cur_node.heuristic_val > child.heuristic_val:
                return frontier.index(cur_node)
    return len(frontier)


def is_node_in_list(list1, node):
    for cur_node in list1:
        if cur_node.state == node.state and cur_node.path_cost == node.path_cost:
            return True
    return False


def constrained(child, constraints):
    for const in constraints:
        if child.get_cur_pos() == const.location and child.path_cost == const.time_step:
            return True
    return False


def calc_all_heuristic(terrain, target):
    heuristic_map = [[0 for i in range(len(terrain))] for j in range(len(terrain[0]))]  # zeros to heuristic_map

    for i in range(len(terrain)):  # blocs to heuristic_map as -1
        for j in range(len(terrain[0])):
            if terrain[i][j] == "@":
                heuristic_map[i][j] = -1

    frontier_map = [target]
    explored_map = []
    current_cost = 0

    while len(frontier_map) > 0:
        temp_frontier = []
        for f_m in frontier_map:
            heuristic_map[f_m[0]][f_m[1]] = current_cost

            explored_map.append(f_m)

            if f_m[0] > 0 and terrain[f_m[0] - 1][f_m[1]] == ".":  # Up
                if [f_m[0] - 1, f_m[1]] not in explored_map and [f_m[0] - 1, f_m[1]] not in temp_frontier:
                    temp_frontier.append([f_m[0] - 1, f_m[1]])
            if f_m[0] < len(terrain) - 1 and terrain[f_m[0] + 1][f_m[1]] == ".":  # down
                if [f_m[0] + 1, f_m[1]] not in explored_map and [f_m[0] + 1, f_m[1]] not in temp_frontier:
                    temp_frontier.append([f_m[0] + 1, f_m[1]])
            if f_m[1] < len(terrain[0]) - 1 and terrain[f_m[0]][f_m[1] + 1] == ".":  # Right
                if [f_m[0] , f_m[1] + 1] not in explored_map and [f_m[0] , f_m[1] + 1] not in temp_frontier:
                    temp_frontier.append([f_m[0], f_m[1] + 1])
            if f_m[1] > 0 and terrain[f_m[0]][f_m[1] - 1] == ".":  # left
                if [f_m[0] , f_m[1] - 1] not in explored_map and [f_m[0] , f_m[1] - 1] not in temp_frontier:
                    temp_frontier.append([f_m[0], f_m[1] - 1])

        frontier_map = temp_frontier

        current_cost += 1

    return heuristic_map

