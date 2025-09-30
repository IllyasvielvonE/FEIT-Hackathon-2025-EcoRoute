# mdvrptw_fixed.py
# Multi-Depot VRPTW (Fixed Fleet): each vehicle starts/ends at its assigned depot.
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
NUM_DEPOTS = 4  # number of depots

random.seed(SEED); np.random.seed(SEED)

def _dist(a,b):
    (x1,y1),(x2,y2)=a,b
    if DISTANCE_METRIC=="manhattan": return abs(x1-x2)+abs(y1-y2)
    return math.hypot(x1-x2,y1-y2)

def create_data():
    # depots at corners (you can change to real coords)
    depots = [(-50,-50),(50,-50),(-50,50),(50,50)][:NUM_DEPOTS]
    customers = [(random.randint(-50,50), random.randint(-50,50)) for _ in range(N_CUSTOMERS)]
    loc = depots + customers
    D, N = len(depots), len(customers); ALL = D + N

    # distance
    M = [[0]*ALL for _ in range(ALL)]
    for i in range(ALL):
        for j in range(ALL):
            if i!=j: M[i][j] = _dist(loc[i], loc[j])

    # demand / service
    demands = [0]*D + [random.randint(1,3) for _ in range(N)]
    service = [0]*D + [SERVICE_TIME]*N

    # time windows: depot all-day; customer by nearest depot travel
    tw = []
    for i in range(ALL):
        if i < D:
            tw.append((0, HORIZON))
        else:
            ready = max(0.0, min(M[d][i] for d in range(D))/SPEED - 10.0)
            tw.append((ready, min(HORIZON, ready + CUSTOMER_WINDOW_SPAN)))

    # vehicles round-robin bind to depots
    starts, ends = [], []
    for v in range(VEHICLES):
        d = v % D
        starts.append(d); ends.append(d)

    data = {
        "depots": depots, "customers": customers, "locations": loc,
        "D": D, "N": N, "ALL": ALL,
        "distance_matrix": M, "demands": demands, "service": service, "time_windows": tw,
        "num_vehicles": VEHICLES,
        "vehicle_capacities": [VEHICLE_CAPACITY]*VEHICLES,
        "vehicle_emissions": [VEHICLE_EF]*VEHICLES,
        "vehicle_fixed": [VEHICLE_FIXED]*VEHICLES,
        "starts": starts, "ends": ends
    }
    return data

def solve_and_plot(data):
    mgr = pywrapcp.RoutingIndexManager(data["ALL"], data["num_vehicles"], data["starts"], data["ends"])
    routing = pywrapcp.RoutingModel(mgr)

    def make_cost(v):
        ef = data["vehicle_emissions"][v]
        def cb(fi, ti):
            i, j = mgr.IndexToNode(fi), mgr.IndexToNode(ti)
            return int(round(ef * data["distance_matrix"][i][j] * SCALE))
        return cb
    for v in range(data["num_vehicles"]):
        idx = routing.RegisterTransitCallback(make_cost(v))
        routing.SetArcCostEvaluatorOfVehicle(idx, v)
        routing.SetFixedCostOfVehicle(int(round(data["vehicle_fixed"][v]*SCALE)), v)

    # capacity
    def dem_cb(from_index):
        return int(data["demands"][mgr.IndexToNode(from_index)])
    d_idx = routing.RegisterUnaryTransitCallback(dem_cb)
    routing.AddDimensionWithVehicleCapacity(d_idx, 0, data["vehicle_capacities"], True, "Cap")
    cap_dim = routing.GetDimensionOrDie("Cap")

    # time = travel/SPEED + service at origin
    def t_cb(fi, ti):
        i, j = mgr.IndexToNode(fi), mgr.IndexToNode(ti)
        t = data["distance_matrix"][i][j]/SPEED + data["service"][i]
        return int(math.ceil(t))
    t_idx = routing.RegisterTransitCallback(t_cb)
    routing.AddDimension(t_idx, 10**6, int(HORIZON), True, "Time")
    time_dim = routing.GetDimensionOrDie("Time")

    # TW
    for n in range(data["ALL"]):
        e,l = data["time_windows"][n]
        time_dim.CumulVar(mgr.NodeToIndex(n)).SetRange(int(e), int(l))

    # search
    p = pywrapcp.DefaultRoutingSearchParameters()
    p.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    p.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    p.time_limit.FromSeconds(TIME_LIMIT_SEC); p.log_search=False

    sol = routing.SolveWithParameters(p)
    if not sol:
        print("❌ Infeasible. Relax windows/horizon or increase vehicles/capacity."); return

    routes_xy = {}; tot_km=tot_co2=0.0
    print("== MDVRPTW (Fixed) Solution ==")
    for v in range(data["num_vehicles"]):
        idx = routing.Start(v)
        if sol.Value(routing.NextVar(idx)) == routing.End(v):
            continue
        ef = data["vehicle_emissions"][v]; Q=data["vehicle_capacities"][v]; F=data["vehicle_fixed"][v]
        veh_km=0.0; veh_co2=F; step=0; poly=[data["locations"][data['starts'][v]]]
        print(f"\n-- Veh {v:02d} start depot {data['starts'][v]} (Q={Q}, EF={ef}, FIX={F}) --")
        while not routing.IsEnd(idx):
            nxt = sol.Value(routing.NextVar(idx))
            i, j = mgr.IndexToNode(idx), mgr.IndexToNode(nxt)
            km = float(data["distance_matrix"][i][j]); dco2 = ef*km
            veh_km += km; veh_co2 += dco2
            arr = sol.Value(time_dim.CumulVar(nxt))
            load = sol.Value(cap_dim.CumulVar(nxt))
            if j >= data["D"]:
                print(f"[{step:02d}] {i:02d}->{j:02d} | Δkm={km:.1f}, ΔCO2={dco2:.2f}, load={load}, t={arr}")
                poly.append(data["locations"][j]); step += 1
            idx = nxt
        poly.append(data["locations"][data['ends'][v]])
        routes_xy[v]=poly; tot_km+=veh_km; tot_co2+=veh_co2
        print(f"Veh {v:02d} summary | km={veh_km:.1f}, CO2={veh_co2:.2f}")
    print(f"\n🌍 Global: km={tot_km:.1f}, CO2={tot_co2:.2f}")

    # plot
    fig, ax = plt.subplots(figsize=(7,7)); fig.patch.set_facecolor("white"); ax.set_facecolor("white")
    for d,(x,y) in enumerate(data["depots"]):
        ax.scatter([x],[y], marker="s", c="black", s=80); ax.text(x+1,y+1,f"D{d}", color="black")
    for i in range(data["N"]):
        n = data["D"]+i; x,y = data["locations"][n]; e,l = data["time_windows"][n]; q=data["demands"][n]
        ax.scatter([x],[y], c="tab:blue", s=60, edgecolors="white", linewidths=2)
        ax.text(x+0.8,y+0.8,f"{n}\nq={q} [{int(e)},{int(l)}]", fontsize=8, color="tab:blue",
                bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.85, lw=0))
    for v,poly in routes_xy.items():
        xs,ys = zip(*poly); ax.plot(xs,ys,linewidth=1.8,alpha=0.9,label=f"Veh#{v:02d}")
    ax.set_title("MDVRPTW (Fixed Fleet)"); ax.grid(True, linestyle=":", alpha=0.3); ax.set_aspect("equal","datalim")
    ax.legend(loc="center left", bbox_to_anchor=(1.02,0.5), fontsize=8, frameon=True); plt.tight_layout()
    plt.savefig("mdvrptw_fixed.png", dpi=150, bbox_inches="tight"); print("🖼 Saved: mdvrptw_fixed.png")

if __name__=="__main__":
    data=create_data(); solve_and_plot(data)