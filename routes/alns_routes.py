# routes/alns_routes.py
from flask import Blueprint, request, jsonify, render_template
from services.osrm_service import osrm_table, osrm_trip
from services.alns_solver import alns_optimize
from services.greedy_solver import greedy_optimize
from services.aco_solver import aco_optimize
from services.local_search import two_opt, three_opt, route_cost

alns_bp = Blueprint("alns", __name__, template_folder="../templates")

# ---- UI ----
@alns_bp.route("/", methods=["GET"])
def index():
    return render_template("index.html")

# ---- Sağlık kontrolü (opsiyonel) ----
@alns_bp.route("/ping", methods=["GET"])
def ping():
    return jsonify({"ok": True})

# ---- Çözüm uç noktası ----
@alns_bp.route("/solve", methods=["POST"])
def solve():
    try:
        data = request.get_json(silent=True) or {}
        depots = data.get("depots", [])
        stops = data.get("stops", [])
        method = str(data.get("method", "alns")).lower()
        improve = data.get("improve")  # "", None, "2opt", "3opt"

        # --- basit doğrulamalar ---
        if not isinstance(depots, list) or not isinstance(stops, list):
            return jsonify({"error": "depots ve stops list olmalı"}), 400
        if len(depots) == 0:
            return jsonify({"error": "En az bir depo/araç başlangıcı gerekir"}), 400

        def _ok_point(p):
            return (
                isinstance(p, (list, tuple))
                and len(p) == 2
                and isinstance(p[0], (int, float))
                and isinstance(p[1], (int, float))
            )

        if not all(_ok_point(p) for p in depots):
            return jsonify({"error": "depots elemanları [enlem, boylam] olmalı"}), 400
        if not all(_ok_point(p) for p in stops):
            return jsonify({"error": "stops elemanları [enlem, boylam] olmalı"}), 400

        all_coords = depots + stops
        depot_count = len(depots)

        # OSRM süre matrisi (saniye)
        mat = osrm_table(all_coords)

        depot_idxs = list(range(depot_count))
        stop_idxs = [depot_count + i for i in range(len(stops))]

        # --- rota üretimi: method seçimi ---
        if method == "greedy":
            routes, cost = greedy_optimize(mat, depot_idxs, stop_idxs)
        elif method == "aco":
            routes, cost = aco_optimize(mat, depot_idxs, stop_idxs)
        else:  # default ALNS
            routes, cost = alns_optimize(mat, depot_idxs, stop_idxs)

        # --- post-processing: 2-opt / 3-opt (GLOBAL, makespan odaklı) ---
        if improve == "2opt":
            routes = two_opt(routes, mat, depot_idxs)
        elif improve == "3opt":
            routes = three_opt(routes, mat, depot_idxs)

        # Makespan (en uzun rota süresi) yeniden hesapla
        costs = [route_cost(mat, depot_idxs[v], routes[v]) for v in range(len(routes))]
        cost = max(costs) if costs else 0

        # OSRM trip ile görselleştirme için geometri al
        trips = []
        for v, idxs in enumerate(routes):
            if not idxs:
                continue  # araç kullanılmamış
            stop_coords = [all_coords[j] for j in idxs]
            trip = osrm_trip(depots[v], stop_coords)
            trips.append({"vehicle": v, "trip": trip})

        return jsonify(
            {
                "method": method,
                "improve": improve,
                "routes": routes,  # indeksler: depots+stops birleşik dizideki stop indexleri
                "cost": cost,      # saniye cinsinden makespan
                "trips": trips,    # her araç için OSRM trip JSON'u
            }
        )

    except Exception as e:
        return jsonify({"error": "internal_error", "message": str(e)}), 500
