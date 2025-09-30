# client2client_pd_open.py
# Multi-Depot OPEN VRP with Pickup&Delivery between clients (A -> B),
# no depot staging: vehicles start/end at any depot (strong sharing),
# capacity + time windows, carbon objective, step-by-step print + plot.

from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import math, random
import numpy as np
import matplotlib.pyplot as plt

# ====== Tunables (aligned with your single_depot_vrptw.py) ======
SEED = 42
DISTANCE_METRIC = "manhattan"   # or "euclid"
SPEED = 1.0
SERVICE_TIME_PICK = 2.0         # service time at pickup A
SERVICE_TIME_DROP = 2.5         # service time at delivery B
HORIZON = 600.0
CUSTOMER_WINDOW_SPAN = 240.0
TIME_LIMIT_SEC = 20
SCALE = 100

# Vehicles (uniform)
VEHICLES = 16
VEHICLE_CAPACITY = 5
VEHICLE_EF = 0.25
VEHICLE_FIXED = 0.0

# Multi-depot
NUM_DEPOTS = 4

# Shipments: number of A->B transfers to create (each is one pickup & delivery pair)
N_SHIPMENTS = 16                # roughly like N_CUSTOMERS in earlier scripts
Q_MIN, Q_MAX = 1, 3             # units per shipment

random.seed(SEED); np.random.seed(SEED)

def _dist(a,b):
    (x1,y1),(x2,y2) = a,b
    if DISTANCE_METRIC == "manhattan":
        return abs(x1-x2)+abs(y1-y2)
    return math.hypot(x1-x2, y1-y2)

def create_data():
    # Depots at corners (you can replace with real coords)
    depots = [(-50,-50),(50,-50),(-50,50),(50,50)][:NUM_DEPOTS]
    D = len(depots)

    # Generate shipment endpoints: A (supply/pickup) and B (demand/delivery)
    # For clarity we generate disjoint coordinates for A set and B set.
    A_nodes = [(random.randint(-60, 0), random.randint(-60, 60)) for _ in range(N_SHIPMENTS)]
    B_nodes = [(random.randint(0, 60),  random.randint(-60, 60)) for _ in range(N_SHIPMENTS)]

    # Quantities
    qty = [random.randint(Q_MIN, Q_MAX) for _ in range(N_SHIPMENTS)]

    # Build node list:
    #   0..(NA-1): A pickups
    #   NA..(NA+NB-1): B deliveries
    #   then virtual starts/ends will be added via RoutingIndexManager (not in coords)
    coords = A_nodes + B_nodes
    NA, NB = len(A_nodes), len(B_nodes)
    assert NA == NB == N_SHIPMENTS
    N_clients = NA + NB

    # Node kinds
    kinds = ["PICK"]*NA + ["DROP"]*NB

    # Distances among clients
    Mcc = [[0]*N_clients for _ in range(N_clients)]
    for i in range(N_clients):
        for j in range(N_clients):
            if i!=j: Mcc[i][j] = _dist(coords[i], coords[j])

    # Depot <-> client distances
    Md2c = [[_dist(depots[d], coords[j]) for j in range(N_clients)] for d in range(D)]
    Mc2d = [[_dist(coords[i], depots[d]) for d in range(D)] for i in range(N_clients)]

    # Time windows:
    #   For pickup A: earliest = nearest-depot travel - small buffer; window width = span
    #   For drop   B: similar logic; you can offset later if you want "A completes before B"
    tw = []
    for i in range(NA):
        ready = max(0.0, min(Md2c[d][i] for d in range(D))/SPEED - 10.0)
        tw.append((ready, min(HORIZON, ready + CUSTOMER_WINDOW_SPAN)))
    for j in range(NB):
        idx = NA + j
        ready = max(0.0, min(Md2c[d][idx] for d in range(D))/SPEED - 10.0)
        tw.append((ready, min(HORIZON, ready + CUSTOMER_WINDOW_SPAN)))

    # Service times per client node
    service = [0.0]*N_clients
    for i in range(NA):       service[i]      = SERVICE_TIME_PICK
    for j in range(NB):       service[NA+j]   = SERVICE_TIME_DROP

    # Build pickup-delivery pairs (one-to-one A_i -> B_i).
    # In practice you could match by distance or demand profile; here 1–1 for clarity.
    pairs = [(i, NA+i) for i in range(N_SHIPMENTS)]

    data = {
        "depots": depots, "D": D,
        "coords": coords, "kinds": kinds, "N": N_clients,
        "Mcc": Mcc, "Md2c": Md2c, "Mc2d": Mc2d,
        "time_windows": tw, "service": service,
        "pairs": pairs, "qty": qty, "NA": NA, "NB": NB,
        "num_vehicles": VEHICLES,
        "vehicle_capacities": [VEHICLE_CAPACITY]*VEHICLES,
        "vehicle_emissions": [VEHICLE_EF]*VEHICLES,
        "vehicle_fixed": [VEHICLE_FIXED]*VEHICLES,
    }
    return data

def solve_and_plot(data):
    N, V, D = data["N"], data["num_vehicles"], data["D"]
    base_n = N
    # Virtual start/end per vehicle (open VRP)
    num_nodes = base_n + 2*V
    starts = [base_n + 2*k     for k in range(V)]
    ends   = [base_n + 2*k + 1 for k in range(V)]

    mgr = pywrapcp.RoutingIndexManager(num_nodes, V, starts, ends)
    routing = pywrapcp.RoutingModel(mgr)

    depot_ids = range(D)
    BIG_M = 10**9

    def is_vs(n): return n >= base_n and ((n-base_n)%2==0)
    def is_ve(n): return n >= base_n and ((n-base_n)%2==1)
    def is_client(n): return n < base_n

    # ---- Cost (carbon): distance * EF; allow unused Start->End = 0 ----
    def make_cost(v):
        ef = data["vehicle_emissions"][v]
        s_node, e_node = starts[v], ends[v]
        def cb(fi, ti):
            i, j = mgr.IndexToNode(fi), mgr.IndexToNode(ti)
            if i==s_node and j==e_node: return 0  # unused vehicle
            if is_vs(i) and is_client(j):
                base = min(data["Md2c"][d][j] for d in depot_ids)
                return int(round(ef * base * SCALE))
            if is_client(i) and is_ve(j):
                base = min(data["Mc2d"][i][d] for d in depot_ids)
                return int(round(ef * base * SCALE))
            if is_client(i) and is_client(j):
                return int(round(ef * data["Mcc"][i][j] * SCALE))
            return BIG_M
        return cb
    for v in range(V):
        idx = routing.RegisterTransitCallback(make_cost(v))
        routing.SetArcCostEvaluatorOfVehicle(idx, v)
        routing.SetFixedCostOfVehicle(int(round(data["vehicle_fixed"][v]*SCALE)), v)

    # ---- Capacity (onboard units) ----
    # load change: +q at pickup A, -q at delivery B
    load_change = [0]*N
    for k,(a,b) in enumerate(data["pairs"]):
        qk = data["qty"][k]
        load_change[a] += qk
        load_change[b] -= qk

    def load_cb(from_index):
        node = mgr.IndexToNode(from_index)
        return int(load_change[node]) if is_client(node) else 0
    l_idx = routing.RegisterUnaryTransitCallback(load_cb)
    routing.AddDimensionWithVehicleCapacity(l_idx, 0, data["vehicle_capacities"], True, "Load")
    load_dim = routing.GetDimensionOrDie("Load")
    # Optional: end empty
    for v in range(V):
        routing.solver().Add(load_dim.CumulVar(routing.End(v)) == 0)

    # ---- Time (travel/SPEED + service at origin) ----
    def time_cb(fi, ti):
        i, j = mgr.IndexToNode(fi), mgr.IndexToNode(ti)
        if is_vs(i) and is_ve(j): return 0
        if is_vs(i) and is_client(j):
            t = min(data["Md2c"][d][j] for d in depot_ids)/SPEED + 0.0
            return int(math.ceil(t))
        if is_client(i) and is_ve(j):
            t = min(data["Mc2d"][i][d] for d in depot_ids)/SPEED + 0.0
            return int(math.ceil(t))
        if is_client(i) and is_client(j):
            t = data["Mcc"][i][j]/SPEED + data["service"][i]
            return int(math.ceil(t))
        return int(1e6)
    t_idx = routing.RegisterTransitCallback(time_cb)
    routing.AddDimension(t_idx, 10**6, int(HORIZON), True, "Time")
    time_dim = routing.GetDimensionOrDie("Time")

    # Time windows for clients + all-day for virtual starts/ends
    for n in range(N):
        e,l = data["time_windows"][n]
        time_dim.CumulVar(mgr.NodeToIndex(n)).SetRange(int(e), int(l))
    for v in range(V):
        time_dim.CumulVar(routing.Start(v)).SetRange(0, int(HORIZON))
        time_dim.CumulVar(routing.End(v)).SetRange(0, int(HORIZON))

    # ---- Pickup&Delivery pairs (A_k -> B_k), same vehicle + precedence ----
    for k,(a,b) in enumerate(data["pairs"]):
        ia, ib = mgr.NodeToIndex(a), mgr.NodeToIndex(b)
        routing.AddPickupAndDelivery(ia, ib)
        routing.solver().Add(routing.VehicleVar(ia) == routing.VehicleVar(ib))
        routing.solver().Add(time_dim.CumulVar(ia) <= time_dim.CumulVar(ib))

    # Search
    p = pywrapcp.DefaultRoutingSearchParameters()
    p.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    p.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    p.time_limit.FromSeconds(TIME_LIMIT_SEC); p.log_search = False

    sol = routing.SolveWithParameters(p)
    if not sol:
        print("❌ Infeasible. Try increasing VEHICLES/CAPACITY, or widen windows/HORIZON.")
        return

    # ---- Print + Plot ----
    routes_xy = {}; tot_km = tot_co2 = 0.0
    print("== Client-to-Client PD (Open/Sharing) Solution ==")
    for v in range(V):
        idx = routing.Start(v)
        if sol.Value(routing.NextVar(idx)) == routing.End(v):  # unused
            continue
        ef = data["vehicle_emissions"][v]; Q = data["vehicle_capacities"][v]; F = data["vehicle_fixed"][v]
        veh_km = 0.0; veh_co2 = F; step = 0; poly = []
        print(f"\n-- Veh {v:02d} (Q={Q}, EF={ef}, FIX={F}) --")
        while not routing.IsEnd(idx):
            nxt = sol.Value(routing.NextVar(idx))
            i, j = mgr.IndexToNode(idx), mgr.IndexToNode(nxt)

            # compute km consistently with cost
            if is_vs(i) and is_client(j):
                km = float(min(data["Md2c"][d][j] for d in range(D)))
            elif is_client(i) and is_ve(j):
                km = float(min(data["Mc2d"][i][d] for d in range(D)))
            elif is_client(i) and is_client(j):
                km = float(data["Mcc"][i][j])
            else:
                km = 0.0
            dco2 = ef * km; veh_km += km; veh_co2 += dco2

            arr = sol.Value(time_dim.CumulVar(nxt))
            load = sol.Value(load_dim.CumulVar(nxt))
            if is_client(j):
                tag = "PICK" if data["kinds"][j] == "PICK" else "DROP"
                print(f"[{step:02d}] {i:02d}->{j:02d} ({tag}) | Δkm={km:.1f}, ΔCO2={dco2:.2f}, load={load}, t={arr}")
                poly.append(data["coords"][j]); step += 1
            idx = nxt

        routes_xy[v] = poly; tot_km += veh_km; tot_co2 += veh_co2
        print(f"Veh {v:02d} summary | km={veh_km:.1f}, CO2={veh_co2:.2f}")
    print(f"\n🌍 Global: km={tot_km:.1f}, CO2={tot_co2:.2f}")

    # Visualization
    fig, ax = plt.subplots(figsize=(7,7))
    fig.patch.set_facecolor("white"); ax.set_facecolor("white")

    # depots
    for d,(x,y) in enumerate(data["depots"]):
        ax.scatter([x],[y], marker="s", c="black", s=80)
        ax.text(x+1,y+1,f"D{d}", color="black")

    # clients
    for n,(x,y) in enumerate(data["coords"]):
        e,l = data["time_windows"][n]
        if data["kinds"][n] == "PICK":
            c = "tab:orange"; lbl = "A/PICK"
        else:
            c = "tab:blue";   lbl = "B/DROP"
        ax.scatter([x],[y], c=c, s=60, edgecolors="white", linewidths=2)
        ax.text(x+0.8, y+0.8, f"{n}\n{lbl} [{int(e)},{int(l)}]",
                fontsize=8, color=c, bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.85, lw=0))

    for v, poly in routes_xy.items():
        if len(poly) >= 2:
            xs, ys = zip(*poly); ax.plot(xs, ys, linewidth=1.8, alpha=0.9, label=f"Veh#{v:02d}")

    ax.set_title("Client-to-Client Transfers (Open/Strong Sharing)")
    ax.grid(True, linestyle=":", alpha=0.3); ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="center left", bbox_to_anchor=(1.02,0.5), fontsize=8, frameon=True); plt.tight_layout()
    plt.savefig("client2client_pd_open.png", dpi=150, bbox_inches="tight")
    print("🖼 Saved: client2client_pd_open.png")

if __name__ == "__main__":
    data = create_data()
    solve_and_plot(data)