from typing import List, Tuple
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from services.local_search import route_cost

def ortools_optimize(matrix: List[List[float]], depot_idxs: List[int], stop_idxs: List[int]) -> Tuple[List[List[int]], float]:
    num_vehicles = len(depot_idxs)
    num_nodes = len(matrix)
    all_nodes = list(range(num_nodes))

    data = {
        "time_matrix": matrix,
        "num_vehicles": num_vehicles,
        "starts": depot_idxs,
        "ends": depot_idxs,
    }

    manager = pywrapcp.RoutingIndexManager(
        len(data["time_matrix"]),
        data["num_vehicles"],
        data["starts"],
        data["ends"],
    )
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(data["time_matrix"][from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    routing.AddDimension(
        transit_callback_index,
        0,
        10**9,
        True,
        "Time",
    )
    time_dimension = routing.GetDimensionOrDie("Time")
    end_vars = [time_dimension.CumulVar(routing.End(v)) for v in range(num_vehicles)]
    routing.AddVariableMinimizedByFinalizer(max(end_vars))

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_params.time_limit.seconds = 20
    search_params.log_search = False

    solution = routing.SolveWithParameters(search_params)
    routes = []

    if not solution:
        return [[] for _ in range(num_vehicles)], 0.0

    for v in range(num_vehicles):
        index = routing.Start(v)
        route = []
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            if node not in depot_idxs:
                route.append(node)
            index = solution.Value(routing.NextVar(index))
        routes.append(route)

    costs = [route_cost(matrix, depot_idxs[v], routes[v]) for v in range(num_vehicles)]
    makespan = max(costs) if costs else 0.0
    return routes, makespan
