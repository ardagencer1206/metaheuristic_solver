def two_opt(route, matrix, start_idx):
    """Verilen rota üzerinde 2-opt iyileştirmesi uygular."""
    best = route
    improved = True
    while improved:
        improved = False
        for i in range(1, len(best)-1):
            for j in range(i+1, len(best)):
                if j-i == 1: continue
                new_route = best[:i] + best[i:j][::-1] + best[j:]
                if route_cost(matrix, start_idx, new_route) < route_cost(matrix, start_idx, best):
                    best = new_route
                    improved = True
    return best

def three_opt(route, matrix, start_idx):
    """Basit 3-opt (swap/reverse) uygular."""
    best = route
    improved = True
    while improved:
        improved = False
        for i in range(len(best) - 2):
            for j in range(i+1, len(best) - 1):
                for k in range(j+1, len(best)):
                    # farklı permütasyonlar denenir
                    segments = [best[:i], best[i:j], best[j:k], best[k:]]
                    candidates = [
                        segments[0] + segments[1][::-1] + segments[2] + segments[3],
                        segments[0] + segments[1] + segments[2][::-1] + segments[3],
                        segments[0] + segments[2] + segments[1] + segments[3],
                    ]
                    for cand in candidates:
                        if route_cost(matrix, start_idx, cand) < route_cost(matrix, start_idx, best):
                            best = cand
                            improved = True
    return best

def route_cost(matrix, start_idx, stop_idxs):
    if not stop_idxs: return 0
    c = matrix[start_idx][stop_idxs[0]]
    for i in range(len(stop_idxs)-1):
        c += matrix[stop_idxs[i]][stop_idxs[i+1]]
    return c
