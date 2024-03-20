
from A_star_algorithm import calc_all_heuristic
from dmapf_solver import DMapfSolver
from dmapf_solver_agent import DMapfSolverAgent
from mapf import MapfProblem
from messages import Message
from read_map_scen import read_map_objects, read_map

import time
import csv

if __name__ == '__main__':
    maps_name = ['random-32-32-10']
    map_n = 0
    min_agents = 2
    max_scenario = 5

    filename_map = f'Maps/{maps_name[map_n]}/{maps_name[map_n]}.map'
    terrain = read_map(filename_map)

    scenario_list = []
    runtime_list = []
    messages_list = []
    sum_of_cost_list = []
    success_list = []
    agent_list = []
    map_list = []

    print(f"Map name: {maps_name[map_n]}")

    for scenario in range(1, max_scenario):

        print(f"Scenario: {scenario}")
        filename_agents = f'Maps/{maps_name[0]}/scen-even/{maps_name[map_n]}-even-{scenario}.scen'
        map_agents = read_map_objects(filename_agents)

        max_numOfAgents = 5
        # max_numOfAgents = len(map_agents) # for all agents in scenario

        all_sources = []
        all_targets = []

        for i in range(max_numOfAgents):
            all_sources.append([map_agents[i].param2, map_agents[i].param1])
            all_targets.append([map_agents[i].param4, map_agents[i].param3])

        for n_a in range(min_agents, max_numOfAgents):
            start_time = time.time()

            sources = all_sources[:n_a]
            targets = all_targets[:n_a]

            agents = [x for x in range(n_a)]

            mapf_problem = MapfProblem(sources, targets, terrain, agents)

            for id_target, target in enumerate(targets):
                mapf_problem.heuristic_maps[id_target] = calc_all_heuristic(terrain, target)

            solver_agents = []

            for agent in agents:
                solver_agents.append(DMapfSolverAgent(agent))

            solver = DMapfSolver()

            sol = solver.solve(mapf_problem, solver_agents)

            print(f"num of agents: {n_a} is done")

            end_time = time.time()

            runtime = end_time - start_time

            runtime_list.append(round(runtime, 3))
            messages_list.append(Message.message_count)
            Message.message_count = 0
            if sol is None:
                sum_of_cost_list.append(-1)
            else:
                sum_of_cost_list.append(sol.cost())
            success_list.append(int(sol is not None))
            agent_list.append(n_a)
            scenario_list.append(scenario)
            map_list.append(maps_name[map_n])

    print(f"Scenario: {scenario_list}")

    print(f"Success: {success_list}")

    print(f"Number of messages: {messages_list}")

    print(f"Runtime: {runtime_list}")

    print(f"Sum of cost: {sum_of_cost_list}")

    print(f"Agents: {agent_list}")

    # Create a list of rows
    rows = zip(map_list, scenario_list, success_list, messages_list, runtime_list, sum_of_cost_list, agent_list)

    # Define the CSV file path
    csv_file = f'results/data-{maps_name[map_n]}.csv'

    # Write the data to the CSV file
    with open(csv_file, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['GridName', 'Scenario', 'Success', 'Number of Messages', 'Runtime', 'Sum of Cost', 'Agents'])
        # Write header
        writer.writerows(rows)  # Write rows

    print("Data saved to data.csv successfully.")