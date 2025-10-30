from flask import Blueprint, request, jsonify, render_template
from services.osrm_service import osrm_table, osrm_trip
from services.alns_solver import alns_optimize
from services.greedy_solver import greedy_optimize
from services.aco_solver import aco_optimize
from services.local_search import two_opt, three_opt

alns_bp = Blueprint("alns", __name__, template_folder="../templates")

@alns_bp.route("/solve", methods=["POST"])
def solve():
    data = request.json
    depots = data["depots"]
    stops  = data["stops"]
    method = data.get("method", "alns")
    improve = data.get("improve", None)  # "2opt" | "3opt" | None

    all_coords = depots + stops
    mat = osrm_table(all_coords)
    depot_idxs = list(range(len(depots)))
    stop_idxs = [len(depots)+i for i in range(len(stops))]

    # önce rota üret
    if method == "greedy":
        routes, cost = greedy_optimize(mat, depot_idxs, stop_idxs)
    elif method == "aco":
        routes, cost = aco_optimize(mat, depot_idxs, stop_idxs)
    else:
        routes, cost = alns_optimize(mat, depot_idxs, stop_idxs)

    # sonra rota başına iyileştirme uygula
    if improve == "2opt":
        for v in range(len(routes)):
            routes[v] = two_opt(routes[v], mat, depot_idxs[v])
    elif improve == "3opt":
        for v in range(len(routes)):
            routes[v] = three_opt(routes[v], mat, depot_idxs[v])

    # tekrar maliyeti hesapla
    costs = [route_cost(mat, depot_idxs[v], routes[v]) for v in range(len(routes))]
    cost = max(costs) if costs else 0

    trips = []
    for v, idxs in enumerate(routes):
        if not idxs: continue
        stop_coords = [all_coords[j] for j in idxs]
        trip = osrm_trip(depots[v], stop_coords)
        trips.append(dict(vehicle=v, trip=trip))

    return jsonify({"method": method, "improve": improve, "routes": routes, "cost": cost, "trips": trips})
