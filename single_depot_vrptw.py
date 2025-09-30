# single_depot_vrptw_en.py
# Single depot + multiple vehicles + capacity + time windows + carbon-emission objective (closed VRPTW)
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import math, random
import numpy as np
import matplotlib.pyplot as plt

# ====== Tunable parameters ======
SEED = 42
N_CUSTOMERS = 32
VEHICLES = 20
DISTANCE_METRIC = "manhattan"   # or "euclid"
SPEED = 1.0                     # distance units per time unit
SERVICE_TIME = 3.0              # service time at each customer
HORIZON = 600.0                 # planning horizon (time units)
CUSTOMER_WINDOW_SPAN = 240.0    # width of each customer time window
TIME_LIMIT_SEC = 20
SCALE = 100                     # scale costs to avoid float issues

# Vehicles (uniform for fair comparison)
VEHICLE_CAPACITY = 5
VEHICLE_EF = 0.25               # kg CO2 per distance unit
VEHICLE_FIXED = 0.0             # fixed emission per used vehicle; set >0 to penalize using more vehicles

random.seed(SEED); np.random.seed(SEED)

def create_data():
    data = {}

    # Single depot at the center; customers randomly placed in [-50, 50]^2
    depot = [(0, 0)]
    customers = [(random.randint(-50, 50), random.randint(-50, 50)) for _ in range(N_CUSTOMERS)]
    loc = depot + customers
    N = len(loc)

    def dist(a, b):
        (x1, y1), (x2, y2) = a, b
        if DISTANCE_METRIC == "manhattan":
            return abs(x1 - x2) + abs(y1 - y2)
        return math.hypot(x1 - x2, y1 - y2)

    # Distance matrix
    M = [[0]*N for _ in range(N)]
    for i in range(N):
        for j in range(N):
            if i != j:
                M[i][j] = dist(loc[i], loc[j])

    # Demands (1~3); depot demand = 0
    demands = [0] + [random.randint(1, 3) for _ in range(N_CUSTOMERS)]

    # Service times (depot=0, customers=SERVICE_TIME)
    service = [0] + [SERVICE_TIME]*N_CUSTOMERS

    # Time windows (distance-aware): ready time = shortest depot->customer travel - buffer
    tw = []
    for i in range(N):
        if i == 0:
            tw.append((0, HORIZON))  # depot open all day
        else:
            ready = max(0.0, M[0][i]/SPEED - 10.0)  # add a small buffer of 10
            end = min(HORIZON, ready + CUSTOMER_WINDOW_SPAN)
            tw.append((ready, end))

    data.update({
        "locations": loc,
        "distance_matrix": M,
        "demands": demands,
        "service": service,
        "time_windows": tw,
        "num_locations": N,
        "depot": 0,
        "num_vehicles": VEHICLES,
        "vehicle_capacities": [VEHICLE_CAPACITY]*VEHICLES,
        "vehicle_emissions": [VEHICLE_EF]*VEHICLES,
        "vehicle_fixed": [VEHICLE_FIXED]*VEHICLES,
    })
    return data

def solve_and_plot(data):
    manager = pywrapcp.RoutingIndexManager(
        data["num_locations"], data["num_vehicles"], data["depot"]
    )
    routing = pywrapcp.RoutingModel(manager)

    # --- Cost (carbon) callback: distance × EF; plus a fixed cost per used vehicle ---
    def make_cost_cb(v):
        ef = data["vehicle_emissions"][v]
        def cb(fi, ti):
            i = manager.IndexToNode(fi); j = manager.IndexToNode(ti)
            d = data["distance_matrix"][i][j]
            return int(round(ef * d * SCALE))
        return cb

    for v in range(data["num_vehicles"]):
        cidx = routing.RegisterTransitCallback(make_cost_cb(v))
        routing.SetArcCostEvaluatorOfVehicle(cidx, v)
        routing.SetFixedCostOfVehicle(int(round(data["vehicle_fixed"][v]*SCALE)), v)

    # --- Capacity dimension ---
    def demand_cb(from_index):
        node = manager.IndexToNode(from_index)
        return int(data["demands"][node])
    demand_idx = routing.RegisterUnaryTransitCallback(demand_cb)
    routing.AddDimensionWithVehicleCapacity(
        demand_idx, 0, data["vehicle_capacities"], True, "Capacity"
    )
    cap_dim = routing.GetDimensionOrDie("Capacity")

    # --- Time dimension (VRPTW): transit = travel time + service at origin node ---
    def time_cb(fi, ti):
        i = manager.IndexToNode(fi); j = manager.IndexToNode(ti)
        travel = data["distance_matrix"][i][j] / SPEED
        return int(math.ceil(travel + data["service"][i]))
    tidx = routing.RegisterTransitCallback(time_cb)
    routing.AddDimension(tidx, 10**6, int(HORIZON), True, "Time")
    time_dim = routing.GetDimensionOrDie("Time")

    # Set time windows for all nodes
    for node in range(data["num_locations"]):
        idx = manager.NodeToIndex(node)
        e, l = data["time_windows"][node]
        time_dim.CumulVar(idx).SetRange(int(math.floor(e)), int(math.ceil(l)))

    # Start/End windows for each vehicle (depot)
    for v in range(data["num_vehicles"]):
        s = routing.Start(v); e = routing.End(v)
        time_dim.CumulVar(s).SetRange(0, int(HORIZON))
        time_dim.CumulVar(e).SetRange(0, int(HORIZON))

    # Search parameters
    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.FromSeconds(TIME_LIMIT_SEC)
    params.log_search = False

    # Print customer demands (pre-solve)
    print("Customer demand list:")
    total_dem = 0
    for i in range(1, data["num_locations"]):
        q = data["demands"][i]
        print(f"  Customer {i:02d}: q={q}")
        total_dem += q
    print(f"→ Total customer demand: {total_dem}, Fleet total capacity: {sum(data['vehicle_capacities'])}\n")

    sol = routing.SolveWithParameters(params)
    if not sol:
        print("❌ Infeasible: relax HORIZON or CUSTOMER_WINDOW_SPAN, or increase vehicles/capacity.")
        total_dem = sum(data["demands"][1:])
        total_cap = sum(data["vehicle_capacities"])
        print(f"- Total demand: {total_dem}, Fleet capacity: {total_cap}")
        return

    # --- Logging & stats ---
    total_km = 0.0; total_co2 = 0.0
    routes_xy = {}

    print("== Single-Depot VRPTW Solution ==")
    for v in range(data["num_vehicles"]):
        idx = routing.Start(v)
        if sol.Value(routing.NextVar(idx)) == routing.End(v):
            # Unused vehicle
            continue

        ef = data["vehicle_emissions"][v]
        Q  = data["vehicle_capacities"][v]
        fixed = data["vehicle_fixed"][v]
        veh_km = 0.0; veh_co2 = fixed
        print(f"\n--- Vehicle {v:02d} (Q={Q}, EF={ef}, FIX={fixed}) ---")

        poly = [data["locations"][0]]
        step = 0
        while not routing.IsEnd(idx):
            next_idx = sol.Value(routing.NextVar(idx))
            i = manager.IndexToNode(idx); j = manager.IndexToNode(next_idx)

            d = data["distance_matrix"][i][j]
            km = float(d)
            dco2 = ef * km
            veh_km += km; veh_co2 += dco2

            arrive = sol.Value(time_dim.CumulVar(next_idx))
            load   = sol.Value(cap_dim.CumulVar(next_idx))
            print(f"[Step {step:02d}] {i:02d} -> {j:02d} | Δkm={km:.1f}, ΔCO2={dco2:.2f}, load={load}, arrive_t={arrive}")
            step += 1

            if j != 0:
                poly.append(data["locations"][j])
            else:
                poly.append(data["locations"][0])
            idx = next_idx

        total_km += veh_km; total_co2 += veh_co2
        routes_xy[v] = poly
        print(f"Vehicle {v:02d} summary | km={veh_km:.1f}, CO2={veh_co2:.2f}")

    print(f"\n🌍 Global: total_km={total_km:.1f}, total_CO2={total_co2:.2f}")

    # --- Visualization ---
    fig, ax = plt.subplots(figsize=(7, 7))
    fig.patch.set_facecolor("white"); ax.set_facecolor("white")
    # Depot
    dep_x, dep_y = data["locations"][0]
    ax.scatter([dep_x], [dep_y], c="black", marker="o", s=120, label="Depot 0")
    ax.text(dep_x+1.0, dep_y+1.0, "0", color="black")

    # Customers + windows + demand(q)
    for i in range(1, data["num_locations"]):
        x, y = data["locations"][i]
        e, l = data["time_windows"][i]
        q = data["demands"][i]  # demand at this customer

        ax.scatter([x], [y], c="tab:blue", s=60, edgecolors="white", linewidths=2)
        ax.text(
            x + 0.8, y + 0.8,
            f"{i}\nq={q}  [{int(e)},{int(l)}]",
            color="tab:blue", fontsize=8,
            bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.8, lw=0)
        )

    # Routes
    for v, poly in routes_xy.items():
        xs, ys = zip(*poly)
        ax.plot(xs, ys, linewidth=1.8, alpha=0.9, label=f"Veh#{v:02d}")

    ax.set_title(f"Single-Depot VRPTW (metric={DISTANCE_METRIC})\nTotal km={total_km:.1f}, Total CO2={total_co2:.2f}")
    ax.grid(True, linestyle=":", alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, labels, loc="center left", bbox_to_anchor=(1.02, 0.5), fontsize=8, frameon=True)
    fig.subplots_adjust(right=0.80)
    plt.tight_layout()
    plt.savefig("single_depot_vrptw.png", dpi=150, bbox_inches="tight")
    print("🖼 Saved: single_depot_vrptw.png")

if __name__ == "__main__":
    data = create_data()
    solve_and_plot(data)