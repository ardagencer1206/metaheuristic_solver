from flask import Blueprint, request, jsonify, render_template
from services.osrm_service import osrm_table, osrm_trip
from services.alns_solver import alns_optimize
from services.greedy_solver import greedy_optimize
from services.aco_solver import aco_optimize

alns_bp = Blueprint("alns", __name__, template_folder="../templates")

@alns_bp.route("/", methods=["GET"])
def index():
    return render_template("index.html")

@alns_bp.route("/solve", methods=["POST"])
def solve():
    data = request.json
    depots = data["depots"]
    stops  = data["stops"]
    method = data.get("method", "alns")   # default ALNS

    all_coords = depots + stops
    mat = osrm_table(all_coords)

    depot_idxs = list(range(len(depots)))
    stop_idxs = [len(depots)+i for i in range(len(stops))]

    if method == "greedy":
        routes, cost = greedy_optimize(mat, depot_idxs, stop_idxs)
    elif method == "aco":
        routes, cost = aco_optimize(mat, depot_idxs, stop_idxs)
    else:
        routes, cost = alns_optimize(mat, depot_idxs, stop_idxs)

    trips = []
    for v, idxs in enumerate(routes):
        if not idxs:
            continue
        stop_coords = [all_coords[j] for j in idxs]
        trip = osrm_trip(depots[v], stop_coords)
        trips.append(dict(vehicle=v, trip=trip))

    return jsonify({"method": method, "routes": routes, "cost": cost, "trips": trips})
