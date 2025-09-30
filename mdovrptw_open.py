# mdovrptw_open.py
# Multi-Depot Open VRPTW (Strong Sharing): vehicles may start at any depot and end at any depot.
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import math, random
import numpy as np
import matplotlib.pyplot as plt

# ====== Tunable parameters (copied from single_depot_vrptw.py) ======
SEED = 42
N_CUSTOMERS = 32
VEHICLES = 20
DISTANCE_METRIC = "manhattan"   # or "euclid"
SPEED = 1.0
SERVICE_TIME = 3.0
HORIZON = 600.0
CUSTOMER_WINDOW_SPAN = 240.0
TIME_LIMIT_SEC = 20
SCALE = 100
VEHICLE_CAPACITY = 5
VEHICLE_EF = 0.25
VEHICLE_FIXED = 0.0

# ====== Multi-depot specific ======
NUM_DEPOTS = 4

random.seed(SEED); np.random.seed(SEED)

def _dist(a,b):
    (x1,y1),(x2,y2)=a,b
    if DISTANCE_METRIC=="manhattan": return abs(x1-x2)+abs(y1-y2)
    return math.hypot(x1-x2,y1-y2)

def create_data():
    depots = [(-50,-50),(50,-50),(-50,50),(50,50)][:NUM_DEPOTS]
    customers = [(random.randint(-50,50), random.randint(-50,50)) for _ in range(N_CUSTOMERS)]
    N = len(customers); D = len(depots)

    # distances
    Mcc  = [[0]*N for _ in range(N)]
    for i in range(N):
        for j in range(N):
            if i!=j: Mcc[i][j] = _dist(customers[i], customers[j])
    Md2c = [[_dist(depots[d], customers[j]) for j in range(N)] for d in range(D)]
    Mc2d = [[_dist(customers[i], depots[d]) for d in range(D)] for i in range(N)]

    demands = [random.randint(1,3) for _ in range(N)]
    service = [SERVICE_TIME]*N

    # TW by nearest depot
    tw_cust = []
    for j in range(N):
        ready = max(0.0, min(Md2c[d][j] for d in range(D))/SPEED - 10.0)
        tw_cust.append((ready, min(HORIZON, ready + CUSTOMER_WINDOW_SPAN)))

    return {
        "depots": depots, "customers": customers, "N": N, "D": D,
        "Mcc": Mcc, "Md2c": Md2c, "Mc2d": Mc2d,
        "demands": demands, "service": service, "tw_cust": tw_cust,
        "num_vehicles": VEHICLES,
        "vehicle_capacities": [VEHICLE_CAPACITY]*VEHICLES,
        "vehicle_emissions": [VEHICLE_EF]*VEHICLES,
        "vehicle_fixed": [VEHICLE_FIXED]*VEHICLES,
    }

def solve_and_plot(data):
    N, V, D = data["N"], data["num_vehicles"], data["D"]
    base_n = N
    num_nodes = base_n + 2*V     # customers + V virtual starts + V virtual ends
    starts = [base_n + 2*k     for k in range(V)]
    ends   = [base_n + 2*k + 1 for k in range(V)]

    mgr = pywrapcp.RoutingIndexManager(num_nodes, V, starts, ends)
    routing = pywrapcp.RoutingModel(mgr)

    depot_ids = list(range(D))
    BIG_M = 10**9
    def is_vs(n): return n >= base_n and ((n-base_n) % 2 == 0)
    def is_ve(n): return n >= base_n and ((n-base_n) % 2 == 1)
    def is_c(n):  return n < base_n

    # cost
    def make_cost(v):
        ef = data["vehicle_emissions"][v]; s_node = starts[v]; e_node = ends[v]
        def cb(fi, ti):
            i, j = mgr.IndexToNode(fi), mgr.IndexToNode(ti)
            if i==s_node and j==e_node: return 0  # unused vehicle
            if i==s_node and is_c(j):
                base = min(data["Md2c"][d][j] for d in depot_ids)
                return int(round(ef * base * SCALE))
            if is_c(i) and j==e_node:
                base = min(data["Mc2d"][i][d] for d in depot_ids)
                return int(round(ef * base * SCALE))
            if is_c(i) and is_c(j):
                return int(round(ef * data["Mcc"][i][j] * SCALE))
            return BIG_M
        return cb
    for v in range(V):
        idx = routing.RegisterTransitCallback(make_cost(v))
        routing.SetArcCostEvaluatorOfVehicle(idx, v)
        routing.SetFixedCostOfVehicle(int(round(data["vehicle_fixed"][v]*SCALE)), v)

    # capacity
    def load_cb(from_index):
        n = mgr.IndexToNode(from_index)
        return int(data["demands"][n]) if is_c(n) else 0
    l_idx = routing.RegisterUnaryTransitCallback(load_cb)
    routing.AddDimensionWithVehicleCapacity(l_idx, 0, data["vehicle_capacities"], True, "Cap")
    cap_dim = routing.GetDimensionOrDie("Cap")

    # time
    def time_cb(fi, ti):
        i, j = mgr.IndexToNode(fi), mgr.IndexToNode(ti)
        if is_vs(i) and is_ve(j): return 0
        if is_vs(i) and is_c(j):
            t = min(data["Md2c"][d][j] for d in depot_ids)/SPEED + 0.0
            return int(math.ceil(t))
        if is_c(i) and is_ve(j):
            t = min(data["Mc2d"][i][d] for d in depot_ids)/SPEED + 0.0
            return int(math.ceil(t))
        if is_c(i) and is_c(j):
            t = data["Mcc"][i][j]/SPEED + data["service"][i]
            return int(math.ceil(t))
        return int(1e6)
    t_idx = routing.RegisterTransitCallback(time_cb)
    routing.AddDimension(t_idx, 10**6, int(HORIZON), True, "Time")
    time_dim = routing.GetDimensionOrDie("Time")

    # TW
    for j in range(base_n):
        e,l = data["tw_cust"][j]
        time_dim.CumulVar(mgr.NodeToIndex(j)).SetRange(int(e), int(l))
    for v in range(V):
        time_dim.CumulVar(routing.Start(v)).SetRange(0, int(HORIZON))
        time_dim.CumulVar(routing.End(v)).SetRange(0, int(HORIZON))

    # search
    p = pywrapcp.DefaultRoutingSearchParameters()
    p.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    p.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    p.time_limit.FromSeconds(TIME_LIMIT_SEC); p.log_search=False

    sol = routing.SolveWithParameters(p)
    if not sol:
        print("❌ Infeasible. Relax windows/horizon or increase vehicles/capacity."); return

    # print + plot
    routes_xy = {}; tot_km=tot_co2=0.0
    print("== MDOVRPTW (Open/Sharing) Solution ==")
    for v in range(V):
        idx = routing.Start(v)
        if sol.Value(routing.NextVar(idx)) == routing.End(v): continue
        ef = data["vehicle_emissions"][v]; Q=data["vehicle_capacities"][v]; F=data["vehicle_fixed"][v]
        veh_km=0.0; veh_co2=F; step=0; poly=[]
        print(f"\n-- Veh {v:02d} (Q={Q}, EF={ef}, FIX={F}) --")
        while not routing.IsEnd(idx):
            nxt = sol.Value(routing.NextVar(idx))
            i, j = mgr.IndexToNode(idx), mgr.IndexToNode(nxt)
            if is_vs(i) and is_c(j):
                km = float(min(data["Md2c"][d][j] for d in depot_ids))
            elif is_c(i) and is_ve(j):
                km = float(min(data["Mc2d"][i][d] for d in depot_ids))
            elif is_c(i) and is_c(j):
                km = float(data["Mcc"][i][j])
            else:
                km = 0.0
            dco2 = ef*km; veh_km += km; veh_co2 += dco2
            arr = sol.Value(time_dim.CumulVar(nxt))
            load = sol.Value(cap_dim.CumulVar(nxt))
            if is_c(j):
                print(f"[{step:02d}] {i:02d}->{j:02d} | Δkm={km:.1f}, ΔCO2={dco2:.2f}, load={load}, t={arr}")
                poly.append(data["customers"][j]); step += 1
            idx = nxt
        routes_xy[v]=poly; tot_km+=veh_km; tot_co2+=veh_co2
        print(f"Veh {v:02d} summary | km={veh_km:.1f}, CO2={veh_co2:.2f}")
    print(f"\n🌍 Global: km={tot_km:.1f}, CO2={tot_co2:.2f}")

    fig, ax = plt.subplots(figsize=(7,7)); fig.patch.set_facecolor("white"); ax.set_facecolor("white")
    for d,(x,y) in enumerate(data["depots"]):
        ax.scatter([x],[y], marker="s", c="black", s=80); ax.text(x+1,y+1,f"D{d}", color="black")
    for j,(x,y) in enumerate(data["customers"]):
        e,l = data["tw_cust"][j]; q = data["demands"][j]
        ax.scatter([x],[y], c="tab:blue", s=60, edgecolors="white", linewidths=2)
        ax.text(x+0.8,y+0.8,f"{j}\nq={q} [{int(e)},{int(l)}]", fontsize=8, color="tab:blue",
                bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.85, lw=0))
    for v,poly in routes_xy.items():
        if len(poly)>=2:
            xs,ys = zip(*poly); ax.plot(xs,ys,linewidth=1.8,alpha=0.9,label=f"Veh#{v:02d}")
    ax.set_title("MDOVRPTW (Open/Strong Sharing)"); ax.grid(True, linestyle=":", alpha=0.3); ax.set_aspect("equal","datalim")
    ax.legend(loc="center left", bbox_to_anchor=(1.02,0.5), fontsize=8, frameon=True); plt.tight_layout()
    plt.savefig("mdovrptw_open.png", dpi=150, bbox_inches="tight"); print("🖼 Saved: mdovrptw_open.png")

if __name__=="__main__":
    data=create_data(); solve_and_plot(data)