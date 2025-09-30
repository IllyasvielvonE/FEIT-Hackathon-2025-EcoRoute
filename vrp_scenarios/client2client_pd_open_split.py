# client2client_pd_open_split.py
# OPEN/strong-sharing + Pickup&Delivery between clients (A->B),
# with automatic split of shipments when qty > vehicle capacity.
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import math, random
import numpy as np
import matplotlib.pyplot as plt

# ===== Tunables (aligned with your VRPTW) =====
SEED = 42
DISTANCE_METRIC = "manhattan"   # or "euclid"
SPEED = 1.0
SERVICE_TIME_PICK = 2.0
SERVICE_TIME_DROP = 2.5
HORIZON = 600.0
CUSTOMER_WINDOW_SPAN = 240.0
TIME_LIMIT_SEC = 20
SCALE = 100

# Vehicles (uniform)
VEHICLES = 16
VEHICLE_CAPACITY = 5           # <= 这个将作为拆分的上限
VEHICLE_EF = 0.25
VEHICLE_FIXED = 0.0

# Multi-depot
NUM_DEPOTS = 4

# Shipments before split (随机生成；若你用手工任务版，思路相同：把每条任务按容量切块）
N_SHIPMENTS = 16
Q_MIN, Q_MAX = 1, 11           # 故意允许 > 容量，便于演示拆分

random.seed(SEED); np.random.seed(SEED)

def _dist(a,b):
    (x1,y1),(x2,y2) = a,b
    if DISTANCE_METRIC == "manhattan": return abs(x1-x2)+abs(y1-y2)
    return math.hypot(x1-x2, y1-y2)

def _split_shipments(A_nodes, B_nodes, qty_list, max_chunk):
    """把每条 A->B 任务按 max_chunk 拆成若干小任务（重复 A/B 坐标与时间窗）。"""
    A2, B2, Q2 = [], [], []
    for a, b, q in zip(A_nodes, B_nodes, qty_list):
        q_left = int(q)
        while q_left > max_chunk:
            A2.append(a); B2.append(b); Q2.append(max_chunk)
            q_left -= max_chunk
        if q_left > 0:
            A2.append(a); B2.append(b); Q2.append(q_left)
    return A2, B2, Q2

def create_data():
    # Depots
    depots = [(-50,-50),(50,-50),(-50,50),(50,50)][:NUM_DEPOTS]
    D = len(depots)

    # 原始 A/B 端与数量（可能 > 车辆容量）
    A_raw = [(random.randint(-60, 0), random.randint(-60, 60)) for _ in range(N_SHIPMENTS)]
    B_raw = [(random.randint(0, 60),  random.randint(-60, 60)) for _ in range(N_SHIPMENTS)]
    Q_raw = [random.randint(Q_MIN, Q_MAX) for _ in range(N_SHIPMENTS)]

    # ====== 关键：按车辆容量自动拆分 ======
    A_nodes, B_nodes, qty = _split_shipments(A_raw, B_raw, Q_raw, VEHICLE_CAPACITY)

    # 组装坐标序列：先放所有 A（PICK），再放所有 B（DROP）
    NA, NB = len(A_nodes), len(B_nodes)
    assert NA == NB
    coords = A_nodes + B_nodes
    N = len(coords)

    kinds = ["PICK"]*NA + ["DROP"]*NB

    # 距离矩阵
    Mcc  = [[0]*N for _ in range(N)]
    for i in range(N):
        for j in range(N):
            if i!=j: Mcc[i][j] = _dist(coords[i], coords[j])
    Md2c = [[_dist(depots[d], coords[j]) for j in range(N)] for d in range(D)]
    Mc2d = [[_dist(coords[i], depots[d]) for d in range(D)] for i in range(N)]

    # 时间窗：A/B 分别按“最近仓→点”的可达最早时间生成
    tw = []
    for i in range(NA):
        ready = max(0.0, min(Md2c[d][i] for d in range(D))/SPEED - 10.0)
        tw.append((ready, min(HORIZON, ready + CUSTOMER_WINDOW_SPAN)))
    for j in range(NB):
        idx = NA + j
        ready = max(0.0, min(Md2c[d][idx] for d in range(D))/SPEED - 10.0)
        tw.append((ready, min(HORIZON, ready + CUSTOMER_WINDOW_SPAN)))

    # 服务时间
    service = [0.0]*N
    for i in range(NA):     service[i]      = SERVICE_TIME_PICK
    for j in range(NB):     service[NA+j]   = SERVICE_TIME_DROP

    # 每个小任务是一对 A_i -> B_i
    pairs = [(i, NA+i) for i in range(NA)]

    return {
        "depots": depots, "D": D,
        "coords": coords, "kinds": kinds, "N": N,
        "Mcc": Mcc, "Md2c": Md2c, "Mc2d": Mc2d,
        "time_windows": tw, "service": service,
        "pairs": pairs, "qty": qty,
        "num_vehicles": VEHICLES,
        "vehicle_capacities": [VEHICLE_CAPACITY]*VEHICLES,
        "vehicle_emissions": [VEHICLE_EF]*VEHICLES,
        "vehicle_fixed": [VEHICLE_FIXED]*VEHICLES,
    }

def solve_and_plot(data, title="Client→Client (Open/Sharing) with Split Deliveries"):
    N, V, D = data["N"], data["num_vehicles"], data["D"]
    base_n = N
    # 虚拟起终点（开放式强共享）
    num_nodes = base_n + 2*V
    starts = [base_n + 2*k     for k in range(V)]
    ends   = [base_n + 2*k + 1 for k in range(V)]

    mgr = pywrapcp.RoutingIndexManager(num_nodes, V, starts, ends)
    routing = pywrapcp.RoutingModel(mgr)

    depot_ids = range(D)
    BIG_M = 10**9
    def is_vs(n): return n >= base_n and ((n-base_n)%2==0)
    def is_ve(n): return n >= base_n and ((n-base_n)%2==1)
    def is_c(n):  return n < base_n

    # 成本（距离×EF），Start->End=0 允许车辆不被使用
    def make_cost(v):
        ef = data["vehicle_emissions"][v]; s_node, e_node = starts[v], ends[v]
        def cb(fi, ti):
            i, j = mgr.IndexToNode(fi), mgr.IndexToNode(ti)
            if i==s_node and j==e_node: return 0
            if is_vs(i) and is_c(j):
                base = min(data["Md2c"][d][j] for d in depot_ids)
                return int(round(ef * base * SCALE))
            if is_c(i) and is_ve(j):
                base = min(data["Mc2d"][i][d] for d in depot_ids)
                return int(round(ef * base * SCALE))
            if is_c(i) and is_c(j):
                return int(round(ef * data["Mcc"][i][j] * SCALE))
            return BIG_M
        return cb
    for v in range(V):
        cidx = routing.RegisterTransitCallback(make_cost(v))
        routing.SetArcCostEvaluatorOfVehicle(cidx, v)
        routing.SetFixedCostOfVehicle(int(round(data["vehicle_fixed"][v]*SCALE)), v)

    # 载荷：在 A 加，在 B 减（每个小任务的数量 ≤ 车容量）
    load_change = [0]*N
    for k,(a,b) in enumerate(data["pairs"]):
        q = int(data["qty"][k]); load_change[a] += q; load_change[b] -= q

    def load_cb(from_index):
        node = mgr.IndexToNode(from_index)
        return int(load_change[node]) if is_c(node) else 0
    lidx = routing.RegisterUnaryTransitCallback(load_cb)
    routing.AddDimensionWithVehicleCapacity(lidx, 0, data["vehicle_capacities"], True, "Load")
    load_dim = routing.GetDimensionOrDie("Load")
    for v in range(V):
        routing.solver().Add(load_dim.CumulVar(routing.End(v)) == 0)

    # 时间（行驶+起点服务）
    def time_cb(fi, ti):
        i, j = mgr.IndexToNode(fi), mgr.IndexToNode(ti)
        if is_vs(i) and is_ve(j): return 0
        if is_vs(i) and is_c(j):
            t = min(data["Md2c"][d][j] for d in depot_ids)/SPEED
            return int(math.ceil(t))
        if is_c(i) and is_ve(j):
            t = min(data["Mc2d"][i][d] for d in depot_ids)/SPEED
            return int(math.ceil(t))
        if is_c(i) and is_c(j):
            t = data["Mcc"][i][j]/SPEED + data["service"][i]
            return int(math.ceil(t))
        return int(1e6)
    tidx = routing.RegisterTransitCallback(time_cb)
    routing.AddDimension(tidx, 10**6, int(HORIZON), True, "Time")
    time_dim = routing.GetDimensionOrDie("Time")

    # 时间窗
    for n in range(N):
        e,l = data["time_windows"][n]
        time_dim.CumulVar(mgr.NodeToIndex(n)).SetRange(int(e), int(l))
    for v in range(V):
        time_dim.CumulVar(routing.Start(v)).SetRange(0, int(HORIZON))
        time_dim.CumulVar(routing.End(v)).SetRange(0, int(HORIZON))

    # P&D：每个拆分后的小任务 A_i->B_i，要求同车 & 先后
    for (a,b) in data["pairs"]:
        ia, ib = mgr.NodeToIndex(a), mgr.NodeToIndex(b)
        routing.AddPickupAndDelivery(ia, ib)
        routing.solver().Add(routing.VehicleVar(ia) == routing.VehicleVar(ib))
        routing.solver().Add(time_dim.CumulVar(ia) <= time_dim.CumulVar(ib))

    # 搜索
    p = pywrapcp.DefaultRoutingSearchParameters()
    p.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    p.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    p.time_limit.FromSeconds(TIME_LIMIT_SEC); p.log_search = False

    sol = routing.SolveWithParameters(p)
    if not sol:
        print("❌ Infeasible. Increase VEHICLES/CAPACITY, widen windows/HORIZON, or lower Q_MAX.")
        return

    # 打印 + 画图
    routes_xy = {}; tot_km = tot_co2 = 0.0
    print("== Client→Client (Open/Sharing) with Split Deliveries ==")
    for v in range(V):
        idx = routing.Start(v)
        if sol.Value(routing.NextVar(idx)) == routing.End(v):  # unused
            continue
        ef = data["vehicle_emissions"][v]; Q=data["vehicle_capacities"][v]; F=data["vehicle_fixed"][v]
        veh_km=0.0; veh_co2=F; step=0; poly=[]
        print(f"\n-- Veh {v:02d} (Q={Q}, EF={ef}, FIX={F}) --")
        while not routing.IsEnd(idx):
            nxt = sol.Value(routing.NextVar(idx))
            i, j = mgr.IndexToNode(idx), mgr.IndexToNode(nxt)
            if is_vs(i) and i!=j and is_c(j):
                km = float(min(data["Md2c"][d][j] for d in range(D)))
            elif is_c(i) and is_ve(j):
                km = float(min(data["Mc2d"][i][d] for d in range(D)))
            elif is_c(i) and is_c(j):
                km = float(data["Mcc"][i][j])
            else:
                km = 0.0
            dco2 = ef*km; veh_km += km; veh_co2 += dco2
            arr = sol.Value(time_dim.CumulVar(nxt))
            load = sol.Value(load_dim.CumulVar(nxt))
            if is_c(j):
                tag = "PICK" if j < (N//2) else "DROP"  # 因为我们是先 A 后 B
                print(f"[{step:02d}] {i:02d}->{j:02d} ({tag}) | Δkm={km:.1f}, ΔCO2={dco2:.2f}, load={load}, t={arr}")
                poly.append(data["coords"][j]); step += 1
            idx = nxt
        routes_xy[v]=poly; tot_km+=veh_km; tot_co2+=veh_co2
        print(f"Veh {v:02d} summary | km={veh_km:.1f}, CO2={veh_co2:.2f}")
    print(f"\n🌍 Global: km={tot_km:.1f}, CO2={tot_co2:.2f}")

    fig, ax = plt.subplots(figsize=(7,7))
    fig.patch.set_facecolor("white"); ax.set_facecolor("white")
    for d,(x,y) in enumerate(data["depots"]):
        ax.scatter([x],[y], marker="s", c="black", s=80); ax.text(x+1,y+1,f"D{d}", color="black")
    for n,(x,y) in enumerate(data["coords"]):
        e,l = data["time_windows"][n]
        c = "tab:orange" if n < (N//2) else "tab:blue"
        lbl = "A/PICK" if n < (N//2) else "B/DROP"
        ax.scatter([x],[y], c=c, s=60, edgecolors="white", linewidths=2)
        ax.text(x+0.8,y+0.8,f"{n}\n{lbl} [{int(e)},{int(l)}]", fontsize=8, color=c,
                bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.85, lw=0))
    for v, poly in routes_xy.items():
        if len(poly)>=2:
            xs,ys = zip(*poly); ax.plot(xs,ys,linewidth=1.8,alpha=0.9,label=f"Veh#{v:02d}")
    ax.set_title(title); ax.grid(True, linestyle=":", alpha=0.3); ax.set_aspect("equal","datalim")
    ax.legend(loc="center left", bbox_to_anchor=(1.02,0.5), fontsize=8, frameon=True); plt.tight_layout()
    plt.savefig("client2client_pd_open_split.png", dpi=150, bbox_inches="tight")
    print("🖼 Saved: client2client_pd_open_split.png")

if __name__ == "__main__":
    data = create_data()
    solve_and_plot(data)