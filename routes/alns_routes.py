# routes/alns_routes.py
from flask import Blueprint, request, jsonify, render_template
from math import isfinite
from services.osrm_service import osrm_table, osrm_trip
from services.alns_solver import alns_optimize
from services.greedy_solver import greedy_optimize
from services.aco_solver import aco_optimize
from services.local_search import two_opt, three_opt, route_cost
from services.ortools_solver import ortools_optimize

# ---- Blueprint ÖNCE tanımlı olmalı ----
alns_bp = Blueprint("alns", __name__, template_folder="../templates")

# ---- UI ----
@alns_bp.route("/", methods=["GET"])
def index():
    return render_template("index.html")

# ---- Sağlık kontrolü ----
@alns_bp.route("/ping", methods=["GET"])
def ping():
    return jsonify({"ok": True})

# ---- Yardımcılar ----
def _ok_point(p):
    return (
        isinstance(p, (list, tuple))
        and len(p) == 2
        and isinstance(p[0], (int, float))
        and isinstance(p[1], (int, float))
    )

def _ok_num(x):
    return isinstance(x, (int, float)) and isfinite(x)

def _makespan_from_trips(trips_json):
    """OSRM trip JSON listesine göre makespan (saniye)."""
    if not trips_json:
        return 0
    durs = [t["trip"]["trips"][0]["duration"] for t in trips_json]
    return max(durs) if durs else 0

def _totals_from_trips(trips_json):
    if not trips_json:
        return {"duration": 0, "distance": 0}
    return {
        "duration": sum(t["trip"]["trips"][0]["duration"] for t in trips_json),
        "distance": sum(t["trip"]["trips"][0]["distance"] for t in trips_json),
    }

def _nn_order(mat, start, nodes):
    """Nearest-neighbor sırası."""
    rem = set(nodes)
    if not rem:
        return []
    cur = min(rem, key=lambda x: mat[start][x])
    seq = [cur]
    rem.remove(cur)
    while rem:
        nxt = min(rem, key=lambda x: mat[cur][x])
        seq.append(nxt)
        rem.remove(nxt)
        cur = nxt
    return seq

def _assign_customers_to_depots(mat, depot_idxs, stop_idxs, demands, depot_stock):
    """Stoğa saygılı en yakın depo ataması. Bölünmüş teslimat yok."""
    V = len(depot_idxs)
    choices = []
    for k, j in enumerate(stop_idxs):
        row = sorted(range(V), key=lambda v: mat[depot_idxs[v]][j])
        choices.append(row)

    stock = depot_stock[:]
    assign = [[] for _ in range(V)]
    for k, j in enumerate(stop_idxs):
        need = demands[k]
        placed = False
        for v in choices[k]:
            if stock[v] >= need:
                stock[v] -= need
                assign[v].append(j)
                placed = True
                break
        if not placed:
            return None
    return assign

# ---- Çözüm uç noktası ----
@alns_bp.route("/solve", methods=["POST"])
def solve():
    try:
        data = request.get_json(silent=True) or {}
        depots = data.get("depots", [])
        stops = data.get("stops", [])
        demands = data.get("demands", [])
        depot_stock = data.get("depot_stock", [])
        vehicle_caps = data.get("vehicle_caps", [])
        method = str(data.get("method", "alns")).lower()
        improve = data.get("improve")  # "", None, "2opt", "3opt"

        # --- doğrulamalar ---
        if not isinstance(depots, list) or not isinstance(stops, list):
            return jsonify({"error": "depots ve stops list olmalı"}), 400
        if len(depots) == 0:
            return jsonify({"error": "En az bir depo gerekir"}), 400
        if not all(_ok_point(p) for p in depots):
            return jsonify({"error": "depots elemanları [enlem, boylam] olmalı"}), 400
        if not all(_ok_point(p) for p in stops):
            return jsonify({"error": "stops elemanları [enlem, boylam] olmalı"}), 400

        N, V = len(stops), len(depots)
        if demands and len(demands) != N:
            return jsonify({"error": "demands uzunluğu stops ile aynı olmalı"}), 400
        if depot_stock and len(depot_stock) != V:
            return jsonify({"error": "depot_stock uzunluğu depots ile aynı olmalı"}), 400
        if vehicle_caps and len(vehicle_caps) != V:
            return jsonify({"error": "vehicle_caps uzunluğu depots ile aynı olmalı"}), 400

        # varsayılanlar
        if not demands:
            demands = [1.0] * N
        if not depot_stock:
            depot_stock = [sum(demands)] * V
        if not vehicle_caps:
            vehicle_caps = [sum(demands)] * V

        if any((not _ok_num(x) or x < 0) for x in demands + depot_stock + vehicle_caps):
            return jsonify({"error": "negatif olmayan sayısal değerler beklenir"}), 400

        all_coords = depots + stops
        depot_idxs = list(range(V))
        stop_idxs = [V + i for i in range(N)]

        # 1) OSRM TABLE
        mat = osrm_table(all_coords)

        # 2) Talep–stok uyumlu depo ataması
        assign = _assign_customers_to_depots(mat, depot_idxs, stop_idxs, demands, depot_stock)
        if assign is None:
            return jsonify({"error": "stok yetersiz. talepler depolara dağıtılamadı"}), 400

        # 3) Depo başına kapasiteye göre alt-turlar
        routed_groups = []  # (vehicle_index, [global_stop_idx,...])
        for v in range(V):
            custs = assign[v]
            cur_load = 0.0
            cur_group = []
            for j in _nn_order(mat, depot_idxs[v], custs):
                dem = demands[j - V]
                if cur_group and cur_load + dem > vehicle_caps[v]:
                    routed_groups.append((v, cur_group))
                    cur_group, cur_load = [], 0.0
                cur_group.append(j)
                cur_load += dem
            if cur_group:
                routed_groups.append((v, cur_group))

        # 4) Her grup için iç sıralama + local search
        final_routes = [[] for _ in range(V)]
        for v, grp in routed_groups:
            # tek depo ile çağır
            if method == "greedy":
                sub_routes, _ = greedy_optimize(mat, [depot_idxs[v]], grp)
            elif method == "aco":
                sub_routes, _ = aco_optimize(mat, [depot_idxs[v]], grp)
            elif method == "ortools":
                # alt-turda tek araç varsayımı
                sub_routes, _ = ortools_optimize(mat, [depot_idxs[v]], grp)
            else:
                sub_routes, _ = alns_optimize(mat, [depot_idxs[v]], grp)

            seq = sub_routes[0] if sub_routes else []

            if improve == "2opt":
                seq = two_opt([seq], mat, [depot_idxs[v]])[0]
            elif improve == "3opt":
                seq = three_opt([seq], mat, [depot_idxs[v]])[0]

            final_routes[v].extend(seq)

        # 5) OSRM TRIP ve metrikler
        base_trips = []
        for v in range(V):
            if not final_routes[v]:
                continue
            stop_coords = [all_coords[j] for j in final_routes[v]]
            base_trips.append({"vehicle": v, "trip": osrm_trip(depots[v], stop_coords)})

        base_mk_real = _makespan_from_trips(base_trips)
        base_costs_matrix = [route_cost(mat, depot_idxs[v], final_routes[v]) for v in range(V)]
        base_mk_matrix = max(base_costs_matrix) if base_costs_matrix else 0
        totals = _totals_from_trips(base_trips)

        return jsonify({
            "method": method,
            "improve": improve,
            "improve_accepted": True,
            "routes": final_routes,
            "makespan_real": base_mk_real,
            "makespan_matrix": base_mk_matrix,
            "trips": base_trips,
            "totals": totals,
            "assignment": {str(v): final_routes[v] for v in range(V)},
            "vehicle_caps": vehicle_caps,
        })

    except Exception as e:
        return jsonify({"error": "internal_error", "message": str(e)}), 500
