def route_cost(matrix, start_idx, stop_idxs):
    if not stop_idxs: 
        return 0
    c = matrix[start_idx][stop_idxs[0]]
    for i in range(len(stop_idxs)-1):
        c += matrix[stop_idxs[i]][stop_idxs[i+1]]
    return c

def greedy_optimize(matrix, depot_idxs, stop_idxs):
    """
    Basit greedy çözüm:
    - Her durağı en yakın araca ata
    - Araç içindeki durakları nearest neighbor ile sırala
    - Amaç: makespan (en uzun rota süresi)
    """
    routes = [[] for _ in depot_idxs]

    # Atama
    for j in stop_idxs:
        best_i, best = 0, float("inf")
        for i in range(len(depot_idxs)):
            d = matrix[depot_idxs[i]][j]
            if d < best:
                best, best_i = d, i
        routes[best_i].append(j)

    # Her rota için nearest neighbor sıralama
    for i in range(len(routes)):
        if len(routes[i]) <= 2:
            continue
        rem = set(routes[i])
        start = depot_idxs[i]
        # start noktasına en yakın durakla başla
        cur = min(rem, key=lambda x: matrix[start][x])
        seq = [cur]
        rem.remove(cur)
        while rem:
            nxt = min(rem, key=lambda x: matrix[cur][x])
            seq.append(nxt)
            rem.remove(nxt)
            cur = nxt
        routes[i] = seq

    # Makespan cost
    costs = [route_cost(matrix, depot_idxs[v], routes[v]) for v in range(len(routes))]
    cost = max(costs) if costs else 0
    return routes, cost
