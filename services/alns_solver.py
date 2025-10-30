import random, math, copy

def route_cost(matrix, start_idx, stop_idxs):
    if not stop_idxs: return 0
    c = matrix[start_idx][stop_idxs[0]]
    for i in range(len(stop_idxs)-1):
        c += matrix[stop_idxs[i]][stop_idxs[i+1]]
    return c

def total_cost(matrix, depot_idxs, routes):
    """Amaç fonksiyonu: en uzun rota süresini minimize et (makespan)."""
    if not routes:
        return 0
    costs = [route_cost(matrix, depot_idxs[v], routes[v]) for v in range(len(routes))]
    return max(costs) if costs else 0


def nearest_init(matrix, depot_idxs, stop_idxs):
    V = len(depot_idxs)
    routes = [[] for _ in range(V)]
    for j in stop_idxs:
        best_i, best = 0, float("inf")
        for i in range(V):
            d = matrix[depot_idxs[i]][j]
            if d < best:
                best, best_i = d, i
        routes[best_i].append(j)
    return routes

def remove_nodes(routes, k):
    rs = copy.deepcopy(routes); removed=[]
    while k>0:
        nonempty = [i for i,r in enumerate(rs) if r]
        if not nonempty: break
        v = random.choice(nonempty)
        pos = random.randrange(len(rs[v]))
        removed.append(rs[v][pos]); rs[v].pop(pos); k-=1
    return rs, removed

def best_insert(matrix, depot_idxs, routes, node):
    best_v, best_pos, best_delta = None, None, float("inf")
    for v in range(len(routes)):
        r = routes[v]
        if not r:
            delta = matrix[depot_idxs[v]][node]
            if delta < best_delta: best_v, best_pos, best_delta = v,0,delta
        else:
            for pos in range(len(r)+1):
                r2 = r[:pos]+[node]+r[pos:]
                delta = route_cost(matrix,depot_idxs[v],r2)-route_cost(matrix,depot_idxs[v],r)
                if delta < best_delta: best_v,best_pos,best_delta=v,pos,delta
    routes[best_v].insert(best_pos,node)

def greedy_repair(matrix,depot_idxs,routes,removed):
    rs=copy.deepcopy(routes)
    for n in removed: best_insert(matrix,depot_idxs,rs,n)
    return rs

def alns_optimize(matrix,depot_idxs,stop_idxs,iters=400):
    routes=nearest_init(matrix,depot_idxs,stop_idxs)
    best=copy.deepcopy(routes); best_cost=total_cost(matrix,depot_idxs,best)
    cur=copy.deepcopy(best); cur_cost=best_cost
    T=best_cost*0.05 if best_cost>0 else 1
    for _ in range(iters):
        k=max(1,int(len(stop_idxs)*0.2))
        rs,removed=remove_nodes(cur,k)
        repaired=greedy_repair(matrix,depot_idxs,rs,removed)
        c=total_cost(matrix,depot_idxs,repaired)
        if c<cur_cost or random.random()<math.exp(-(c-cur_cost)/T):
            cur,cur_cost=repaired,c
            if c<best_cost: best,best_cost=copy.deepcopy(repaired),c
        T*=0.995
    return best,best_cost
