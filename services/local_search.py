def route_cost(matrix, start_idx, stop_idxs):
    """Bir rotanın maliyetini hesapla."""
    if not stop_idxs:
        return 0
    c = matrix[start_idx][stop_idxs[0]]
    for i in range(len(stop_idxs) - 1):
        c += matrix[stop_idxs[i]][stop_idxs[i + 1]]
    return c


def improve_longest_with_2opt(routes, matrix, depot_idxs):
    """Tüm araçlar içinde en uzun rotayı seçip 2-opt uygula."""
    if not routes:
        return routes

    # en uzun rotayı bul
    costs = [route_cost(matrix, depot_idxs[v], routes[v]) for v in range(len(routes))]
    v_long = max(range(len(routes)), key=lambda v: costs[v])

    # 2-opt uygula
    best = routes[v_long]
    improved = True
    while improved:
        improved = False
        for i in range(1, len(best) - 1):
            for j in range(i + 1, len(best)):
                if j - i == 1:
                    continue
                new_route = best[:i] + best[i:j][::-1] + best[j:]
                if route_cost(matrix, depot_idxs[v_long], new_route) < route_cost(matrix, depot_idxs[v_long], best):
                    best = new_route
                    improved = True
    routes[v_long] = best
    return routes


def improve_longest_with_3opt(routes, matrix, depot_idxs):
    """Tüm araçlar içinde en uzun rotayı seçip 3-opt uygula."""
    if not routes:
        return routes

    costs = [route_cost(matrix, depot_idxs[v], routes[v]) for v in range(len(routes))]
    v_long = max(range(len(routes)), key=lambda v: costs[v])

    best = routes[v_long]
    improved = True
    while improved:
        improved = False
        for i in range(len(best) - 2):
            for j in range(i + 1, len(best) - 1):
                for k in range(j + 1, len(best)):
                    segments = [best[:i], best[i:j], best[j:k], best[k:]]
                    candidates = [
                        segments[0] + segments[1][::-1] + segments[2] + segments[3],
                        segments[0] + segments[1] + segments[2][::-1] + segments[3],
                        segments[0] + segments[2] + segments[1] + segments[3],
                    ]
                    for cand in candidates:
                        if route_cost(matrix, depot_idxs[v_long], cand) < route_cost(matrix, depot_idxs[v_long], best):
                            best = cand
                            improved = True
    routes[v_long] = best
    return routes
