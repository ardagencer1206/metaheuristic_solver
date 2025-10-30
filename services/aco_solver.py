import random
import math

def route_cost(matrix, start_idx, stop_idxs):
    if not stop_idxs:
        return 0
    c = matrix[start_idx][stop_idxs[0]]
    for i in range(len(stop_idxs)-1):
        c += matrix[stop_idxs[i]][stop_idxs[i+1]]
    return c

def aco_optimize(matrix, depot_idxs, stop_idxs, ants=20, iterations=50, alpha=1.0, beta=2.0, rho=0.5):
    """
    Ant Colony Optimization ile çoklu araç rotalama (çok basitleştirilmiş).
    Varsayılan hedef: makespan (en uzun rota süresini minimize et).
    """

    n = len(matrix)
    pheromone = [[1.0 for _ in range(n)] for _ in range(n)]

    best_routes, best_cost = None, float("inf")

    for _ in range(iterations):
        all_routes = []
        all_costs = []

        for _ in range(ants):
            # her karınca için rotaları oluştur
            routes = [[] for _ in depot_idxs]
            unvisited = set(stop_idxs)

            while unvisited:
                for v in range(len(depot_idxs)):
                    if not unvisited:
                        break
                    current = depot_idxs[v] if not routes[v] else routes[v][-1]

                    # seçim olasılıklarını hesapla
                    probs = []
                    denom = 0
                    for j in unvisited:
                        tau = pheromone[current][j] ** alpha
                        eta = (1.0 / (matrix[current][j] + 1e-6)) ** beta
                        val = tau * eta
                        denom += val
                        probs.append((j, val))

                    if denom == 0:
                        j = random.choice(list(unvisited))
                    else:
                        r = random.random()
                        cum = 0
                        j = None
                        for node, val in probs:
                            cum += val / denom
                            if r <= cum:
                                j = node
                                break
                        if j is None:
                            j = probs[-1][0]

                    routes[v].append(j)
                    unvisited.remove(j)

            # cost = makespan
            costs = [route_cost(matrix, depot_idxs[v], routes[v]) for v in range(len(routes))]
            cost = max(costs) if costs else 0
            all_routes.append(routes)
            all_costs.append(cost)

            if cost < best_cost:
                best_cost, best_routes = cost, routes

        # feromon güncelleme
        for i in range(n):
            for j in range(n):
                pheromone[i][j] *= (1 - rho)

        for routes, cost in zip(all_routes, all_costs):
            for v in range(len(routes)):
                seq = [depot_idxs[v]] + routes[v]
                for a, b in zip(seq[:-1], seq[1:]):
                    pheromone[a][b] += 1.0 / (1.0 + cost)

    return best_routes, best_cost
