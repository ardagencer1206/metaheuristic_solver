# services/local_search.py
from copy import deepcopy

def route_cost(matrix, start_idx, stop_idxs):
    """Tek bir aracın rota maliyeti (süre/mesafe matrisine göre)."""
    if not stop_idxs:
        return 0.0
    c = matrix[start_idx][stop_idxs[0]]
    for i in range(len(stop_idxs) - 1):
        c += matrix[stop_idxs[i]][stop_idxs[i + 1]]
    return float(c)

def _route_costs(matrix, depot_idxs, routes):
    return [route_cost(matrix, depot_idxs[v], routes[v]) for v in range(len(routes))]

def _makespan(costs):
    return max(costs) if costs else 0.0

# -------- 2-OPT (global makespan odaklı) --------
def two_opt(routes, matrix, depot_idxs, max_iter=200):
    """
    En uzun rota üzerinde 2-opt best-improvement uygular.
    Hamle, makespan'i düşürüyorsa kabul edilir. İyileşme yoksa durur.
    """
    cur = deepcopy(routes)
    it = 0
    while it < max_iter:
        it += 1
        if not cur:
            break

        costs = _route_costs(matrix, depot_idxs, cur)
        v_long = max(range(len(cur)), key=lambda v: costs[v]) if costs else 0
        base_mk = _makespan(costs)

        r = cur[v_long]
        if len(r) < 3:
            # 2-opt yapacak uzunluk yok
            break

        best_route = r
        best_mk = base_mk
        improved = False

        # 2-opt: iki kesme noktası (i, j), i<j
        for i in range(1, len(r) - 1):
            for j in range(i + 1, len(r)):
                if j - i == 1:
                    continue  # bitişik kenarı ters çevirmek anlamsız
                cand = r[:i] + r[i:j][::-1] + r[j:]

                # sadece uzun rota maliyetini yeniden hesaplamak yeter
                new_costs = costs[:]  # shallow copy
                new_costs[v_long] = route_cost(matrix, depot_idxs[v_long], cand)
                mk = _makespan(new_costs)

                if mk < best_mk:
                    best_mk = mk
                    best_route = cand
                    improved = True

        if improved:
            cur[v_long] = best_route
        else:
            break  # daha fazla makespan iyileştirmesi yok

    return cur

# -------- 3-OPT (global makespan odaklı) --------
def three_opt(routes, matrix, depot_idxs, max_iter=100):
    """
    En uzun rota üzerinde basit 3-opt varyantları dener.
    Hamle, makespan'i düşürürse kabul edilir. İyileşme yoksa durur.
    """
    cur = deepcopy(routes)
    it = 0
    while it < max_iter:
        it += 1
        if not cur:
            break

        costs = _route_costs(matrix, depot_idxs, cur)
        v_long = max(range(len(cur)), key=lambda v: costs[v]) if costs else 0
        base_mk = _makespan(costs)

        r = cur[v_long]
        if len(r) < 4:
            break  # 3-opt için yeterince düğüm yok

        best_route = r
        best_mk = base_mk
        improved = False

        # 3-opt: üç kesme noktası (i, j, k), i<j<k
        for i in range(0, len(r) - 2):
            for j in range(i + 1, len(r) - 1):
                for k in range(j + 1, len(r)):
                    A = r[:i]
                    B = r[i:j]
                    C = r[j:k]
                    D = r[k:]

                    # Birkaç yaygın 3-opt yeniden bağlama adayı
                    candidates = [
                        A + B[::-1] + C + D,
                        A + B + C[::-1] + D,
                        A + C + B + D,
                        A + C[::-1] + B + D,
                        A + B[::-1] + C[::-1] + D,
                    ]

                    for cand in candidates:
                        new_costs = costs[:]
                        new_costs[v_long] = route_cost(matrix, depot_idxs[v_long], cand)
                        mk = _makespan(new_costs)
                        if mk < best_mk:
                            best_mk = mk
                            best_route = cand
                            improved = True

        if improved:
            cur[v_long] = best_route
        else:
            break

    return cur
