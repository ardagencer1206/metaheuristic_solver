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

# ---- Yardımcılar ----
def _ok_point(p):
    return (
        isinstance(p, (list, tuple)) and len(p) == 2
        and isinstance(p[0], (int, float)) and isinstance(p[1], (int, float))
    )

def _makespan_from_trips(trips_json):
    """OSRM trip JSON listesine göre makespan (sn) döndürür."""
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

# ---- Çözüm uç noktası ----
@alns_bp.route("/solve", methods=["POST"])
def solve():
    try:
        data = request.get_json(silent=True) or {}
        depots = data.get("depots", [])
        stops = data.get("stops", [])
        method = str(data.get("method", "alns")).lower()
        improve = data.get("improve")  # "", None, "2opt", "3opt"

        # --- doğrulamalar ---
        if not isinstance(depots, list) or not isinstance(stops, list):
            return jsonify({"error": "depots ve stops list olmalı"}), 400
        if len(depots) == 0:
            return jsonify({"error": "En az bir depo/araç başlangıcı gerekir"}), 400
        if not all(_ok_point(p) for p in depots):
            return jsonify({"error": "depots elemanları [enlem, boylam] olmalı"}), 400
        if not all(_ok_point(p) for p in stops):
            return jsonify({"error": "stops elemanları [enlem, boylam] olmalı"}), 400

        all_coords = depots + stops
        depot_count = len(depots)

        # 1) OSRM TABLE (saniye cinsinden süre matrisi)
        mat = osrm_table(all_coords)
        depot_idxs = list(range(depot_count))
        stop_idxs = [depot_count + i for i in range(len(stops))]

        # 2) ÇÖZÜCÜ
        if method == "greedy":
            routes, _ = greedy_optimize(mat, depot_idxs, stop_idxs)
        elif method == "aco":
            routes, _ = aco_optimize(mat, depot_idxs, stop_idxs)
        else:  # default ALNS
            routes, _ = alns_optimize(mat, depot_idxs, stop_idxs)

        # 3) BAZ (GERÇEK) TRIP'LER ve METRİKLER
        base_trips = []
        for v, idxs in enumerate(routes):
            if not idxs:
                continue  # araç kullanılmamış
            stop_coords = [all_coords[j] for j in idxs]
            base_trips.append({"vehicle": v, "trip": osrm_trip(depots[v], stop_coords)})
        base_mk_real = _makespan_from_trips(base_trips)

        # (isteğe bağlı) matrise göre de raporlayalım
        base_costs_matrix = [route_cost(mat, depot_idxs[v], routes[v]) for v in range(len(routes))]
        base_mk_matrix = max(base_costs_matrix) if base_costs_matrix else 0

        final_routes = routes
        final_trips = base_trips
        final_mk_real = base_mk_real
        final_mk_matrix = base_mk_matrix
        improve_accepted = False

        # 4) (OPSİYONEL) LOCAL SEARCH — yalnızca GERÇEK makespan azalırsa kabul et
        cand_routes = routes
        if improve == "2opt":
            cand_routes = two_opt(routes, mat, depot_idxs)   # global, en uzun rota odaklı
        elif improve == "3opt":
            cand_routes = three_opt(routes, mat, depot_idxs) # global, en uzun rota odaklı

        if cand_routes is not routes:  # iyileştirme bir şey değiştirdiyse
            cand_trips = []
            for v, idxs in enumerate(cand_routes):
                if not idxs:
                    continue
                stop_coords = [all_coords[j] for j in idxs]
                cand_trips.append({"vehicle": v, "trip": osrm_trip(depots[v], stop_coords)})
            cand_mk_real = _makespan_from_trips(cand_trips)

            # Kabul kuralı: GERÇEK (OSRM) makespan düşmeli
            if cand_mk_real < base_mk_real:
                improve_accepted = True
                final_routes = cand_routes
                final_trips = cand_trips
                final_mk_real = cand_mk_real
                # matrise göre de güncelle
                cand_costs_matrix = [route_cost(mat, depot_idxs[v], cand_routes[v]) for v in range(len(cand_routes))]
                final_mk_matrix = max(cand_costs_matrix) if cand_costs_matrix else 0

        # 5) ÖZETLE VE DÖN
        totals = _totals_from_trips(final_trips)

        return jsonify({
            "method": method,
            "improve": improve,
            "improve_accepted": improve_accepted,
            "routes": final_routes,              # birleşik dizideki stop indexleri
            "cost": final_mk_real,               # UI için makespan (OSRM) sn
            "makespan_real": final_mk_real,      # OSRM gerçek makespan
            "makespan_matrix": final_mk_matrix,  # table'a göre makespan
            "trips": final_trips,                # her araç için OSRM trip JSON'u
            "totals": totals
        })

    except Exception as e:
        return jsonify({"error": "internal_error", "message": str(e)}), 500
