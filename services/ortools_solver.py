from typing import List, Tuple
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

def ortools_optimize(matrix: List[List[float]], depot_idxs: List[int], stop_idxs: List[int],
                     demands: List[float]=None, vehicle_caps: List[float]=None) -> Tuple[List[List[int]], float]:
    # Bu fonksiyon tek depo/tek araç listesi ile çağrılıyor: depot_idxs uzunluğu == 1 varsayımı
    num_vehicles = len(depot_idxs)
    if num_vehicles != 1:
        # mevcut çağrı kalıbında her alt-tur için tek araç geliyor
        pass

    nodes = depot_idxs + stop_idxs
    index_map = {n:i for i,n in enumerate(nodes)}
    tm = [[matrix[a][b] for b in nodes] for a in nodes]

    data = {
        "time_matrix": tm,
        "num_vehicles": 1,
        "starts": [0],
        "ends": [0],
        "demands": [0.0] + [0.0 if demands is None else float(demands[i]) for i in range(len(stop_idxs))],
        "vehicle_caps": [10**12 if vehicle_caps is None else float(vehicle_caps[0])],
    }

    manager = pywrapcp.RoutingIndexManager(len(data["time_matrix"]), data["num_vehicles"], data["starts"], data["ends"])
    routing = pywrapcp.RoutingModel(manager)

    def time_cb(from_index, to_index):
        a = manager.IndexToNode(from_index); b = manager.IndexToNode(to_index)
        return int(data["time_matrix"][a][b])

    tcb = routing.RegisterTransitCallback(time_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(tcb)

    # Kapasite boyutu
    def demand_cb(index):
        node = manager.IndexToNode(index)
        return int(data["demands"][node])
    dcb = routing.RegisterUnaryTransitCallback(demand_cb)
    routing.AddDimensionWithVehicleCapacity(
        dcb, 0, [int(data["vehicle_caps"][0])], True, "Capacity"
    )

    # Zaman boyutu ve makespan finalizer
    routing.AddDimension(tcb, 0, 10**12, True, "Time")
    time_dimension = routing.GetDimensionOrDie("Time")
    end_var = time_dimension.CumulVar(routing.End(0))
    routing.AddVariableMinimizedByFinalizer(end_var)

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.seconds = 10

    sol = routing.SolveWithParameters(params)
    if not sol:
        return [[]], 0.0

    # çözümü topla
    route = []
    idx = routing.Start(0)
    while not routing.IsEnd(idx):
        node = manager.IndexToNode(idx)
        if node != 0:  # depot
            # geri global indeks
            route.append(nodes[node])
        idx = sol.Value(routing.NextVar(idx))

    # makespan
    def _rc(start_idx, seq):
        if not seq: return 0.0
        c = matrix[start_idx][seq[0]]
        for i in range(len(seq)-1): c += matrix[seq[i]][seq[i+1]]
        return float(c)
    return [route], _rc(depot_idxs[0], route)
