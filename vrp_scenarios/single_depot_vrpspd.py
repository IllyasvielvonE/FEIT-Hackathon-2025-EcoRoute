# single_depot_vrpspd.py
# Single depot, multi-vehicle, capacity + time windows, with BOTH deliveries and pickups (swap at predicted EOL sites)

from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import math

import matplotlib.pyplot as plt

# -------- bridge: reuse VRPTW data (same tunables / windows / vehicles) --------
def build_vrpspd_from_vrptw(base, eol_ratio=0.4, service_time_deliver=3.0, service_time_pickup=3.0):
    import random
    random.seed(base.get("seed", 42))

    DEPOT = base["depot"]  # assume 0
    coords_base = base["locations"]
    N_cust = base["num_locations"] - 1
    deliver_q = list(base["demands"][1:])      # same demands as VRPTW deliveries
    tw_base   = list(base["time_windows"])     # reuse windows
    veh_caps  = base["vehicle_capacities"]
    veh_ef    = base["vehicle_emissions"]
    veh_fix   = base["vehicle_fixed"]
    V         = base["num_vehicles"]

    # choose EOL subset
    k = max(1, int(round(eol_ratio * N_cust)))
    import random
    eol_idx = set(random.sample(range(N_cust), k))
    pickup_q = [deliver_q[i] if i in eol_idx else 0 for i in range(N_cust)]

    # ----- build nodes -----
    coords = []
    node_kind = []  # 'DEPOT','CUST_DEL','CUST_PU','DP','DR'
    coords.append(coords_base[0]); node_kind.append("DEPOT")
    cust_del_node=[]; cust_pick_node=[]; dp_node=[]; dr_node=[]

    # customer deliver nodes
    for i in range(N_cust):
        coords.append(coords_base[i+1]); node_kind.append("CUST_DEL")
        cust_del_node.append(len(coords)-1)
    # customer pickup nodes (same coords, only for EOL)
    for i in range(N_cust):
        if pickup_q[i] > 0:
            coords.append(coords_base[i+1]); node_kind.append("CUST_PU")
            cust_pick_node.append(len(coords)-1)
        else:
            cust_pick_node.append(-1)
    # depot-pick copies (load outbound)
    for i in range(N_cust):
        coords.append(coords_base[0]); node_kind.append("DP")
        dp_node.append(len(coords)-1)
    # depot-drop copies (drop inbound old)
    for i in range(N_cust):
        if pickup_q[i] > 0:
            coords.append(coords_base[0]); node_kind.append("DR")
            dr_node.append(len(coords)-1)
        else:
            dr_node.append(-1)

    N_nodes = len(coords)

    # distance matrix: reuse base metric implicitly by rebuilding from coords with same geometry
    def _dist(a,b): (x1,y1),(x2,y2)=a,b; return abs(x1-x2)+abs(y1-y2)  # manhattan like base
    M = [[0]*N_nodes for _ in range(N_nodes)]
    for i in range(N_nodes):
        for j in range(N_nodes):
            if i!=j: M[i][j] = _dist(coords[i], coords[j])

    # time windows: depot from base; customers reuse base; DP=[0,0]; DR=[0,H]
    H = int(tw_base[0][1])
    tw = [(0, H)]
    for i in range(N_cust): tw.append(tw_base[i+1])              # CUST_DEL
    for i in range(N_cust):                                       # CUST_PU
        if cust_pick_node[i] != -1: tw.append(tw_base[i+1])
    for _ in range(N_cust): tw.append((0,0))                      # DP
    for i in range(N_cust):                                       # DR
        if dr_node[i] != -1: tw.append((0,H))

    # service times
    service = [0.0]*N_nodes
    # if base has per-node service, reuse the customer service time
    base_service = base.get("service", None)
    st_del = (base_service[1] if base_service and len(base_service)>1 else service_time_deliver)
    st_pu  = service_time_pickup
    for i in range(N_cust): service[cust_del_node[i]] = st_del
    for i in range(N_cust):
        if cust_pick_node[i] != -1: service[cust_pick_node[i]] = st_pu

    # load change (+ at pickups, - at deliveries)
    load_change = [0]*N_nodes
    for i in range(N_cust):
        qd = deliver_q[i]; qp = pickup_q[i]
        load_change[dp_node[i]]       = +qd
        load_change[cust_del_node[i]] = -qd
        if cust_pick_node[i] != -1: load_change[cust_pick_node[i]] = +qp
        if dr_node[i] != -1:        load_change[dr_node[i]]       = -qp

    return {
        "coords": coords,
        "distance_matrix": M,
        "time_windows": tw,
        "service": service,
        "load_change": load_change,
        "depot": 0,
        "num_nodes": N_nodes,
        "num_vehicles": V,
        "vehicle_capacities": veh_caps,
        "vehicle_emissions": veh_ef,
        "vehicle_fixed": veh_fix,
        "cust_del_node": cust_del_node,
        "cust_pick_node": cust_pick_node,
        "dp_node": dp_node,
        "dr_node": dr_node,
        "deliver_q": deliver_q,
        "pickup_q": pickup_q,
        "node_kind": node_kind,
        "SCALE": base.get("SCALE", 100),
        "TIME_LIMIT_SEC": base.get("TIME_LIMIT_SEC", 20),
    }

def solve_and_plot(data):
    SCALE = data.get("SCALE", 100)
    TIME_LIMIT_SEC = data.get("TIME_LIMIT_SEC", 20)

    manager = pywrapcp.RoutingIndexManager(data["num_nodes"], data["num_vehicles"], data["depot"])
    routing = pywrapcp.RoutingModel(manager)

    # cost = distance * EF + fixed per used vehicle
    def make_cost_cb(v):
        ef = data["vehicle_emissions"][v]
        def cb(fi, ti):
            i = manager.IndexToNode(fi); j = manager.IndexToNode(ti)
            return int(round(ef * data["distance_matrix"][i][j] * SCALE))
        return cb
    for v in range(data["num_vehicles"]):
        cidx = routing.RegisterTransitCallback(make_cost_cb(v))
        routing.SetArcCostEvaluatorOfVehicle(cidx, v)
        routing.SetFixedCostOfVehicle(int(round(data["vehicle_fixed"][v]*SCALE)), v)

    # capacity
    def load_cb(from_index):
        node = manager.IndexToNode(from_index)
        return int(data["load_change"][node])
    load_idx = routing.RegisterUnaryTransitCallback(load_cb)
    routing.AddDimensionWithVehicleCapacity(load_idx, 0, data["vehicle_capacities"], True, "Load")
    load_dim = routing.GetDimensionOrDie("Load")
    for v in range(data["num_vehicles"]):
        routing.solver().Add(load_dim.CumulVar(routing.End(v)) == 0)

    # time
    def time_cb(fi, ti):
        i = manager.IndexToNode(fi); j = manager.IndexToNode(ti)
        travel = data["distance_matrix"][i][j]
        return int(math.ceil(travel + data["service"][i]))
    tidx = routing.RegisterTransitCallback(time_cb)
    routing.AddDimension(tidx, 10**6, max(int(data["time_windows"][0][1]), 0), True, "Time")
    time_dim = routing.GetDimensionOrDie("Time")

    for node in range(data["num_nodes"]):
        idx = manager.NodeToIndex(node)
        e, l = data["time_windows"][node]
        time_dim.CumulVar(idx).SetRange(int(e), int(l))

    # shipments
    for i, n_del in enumerate(data["cust_del_node"]):
        a = manager.NodeToIndex(data["dp_node"][i])
        b = manager.NodeToIndex(n_del)
        routing.AddPickupAndDelivery(a, b)
        routing.solver().Add(routing.VehicleVar(a) == routing.VehicleVar(b))
        routing.solver().Add(time_dim.CumulVar(a) <= time_dim.CumulVar(b))

    for i, n_pick in enumerate(data["cust_pick_node"]):
        if n_pick == -1: continue
        a = manager.NodeToIndex(n_pick)
        b = manager.NodeToIndex(data["dr_node"][i])
        routing.AddPickupAndDelivery(a, b)
        routing.solver().Add(routing.VehicleVar(a) == routing.VehicleVar(b))
        routing.solver().Add(time_dim.CumulVar(a) <= time_dim.CumulVar(b))
        # ensure deliver-before-pick at the same site (swap)
        n_del = data["cust_del_node"][i]
        routing.solver().Add(time_dim.CumulVar(a) >= time_dim.CumulVar(manager.NodeToIndex(n_del)))

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.FromSeconds(TIME_LIMIT_SEC)
    params.log_search = False

    sol = routing.SolveWithParameters(params)
    if not sol:
        print("❌ Infeasible. Relax base HORIZON/windows or reduce EOL ratio."); return

    total_km = 0.0; total_co2 = 0.0; routes_xy = {}
    print("== Single-Depot VRPSPD Solution (reusing VRPTW tunables) ==")
    for v in range(data["num_vehicles"]):
        idx = routing.Start(v)
        if sol.Value(routing.NextVar(idx)) == routing.End(v): continue
        ef = data["vehicle_emissions"][v]; Q = data["vehicle_capacities"][v]; fixed = data["vehicle_fixed"][v]
        veh_km = 0.0; veh_co2 = fixed
        print(f"\n--- Vehicle {v:02d} (Q={Q}, EF={ef}, FIX={fixed}) ---")
        poly = [data["coords"][data["depot"]]]; step = 0
        while not routing.IsEnd(idx):
            nxt = sol.Value(routing.NextVar(idx))
            i = manager.IndexToNode(idx); j = manager.IndexToNode(nxt)
            d = data["distance_matrix"][i][j]; km = float(d); dco2 = ef*km
            veh_km += km; veh_co2 += dco2
            t_arr = sol.Value(time_dim.CumulVar(nxt)); load = sol.Value(load_dim.CumulVar(nxt))
            kind_j = data["node_kind"][j]
            if kind_j not in ("DP","DR"):
                print(f"[Step {step:02d}] {i:02d} -> {j:02d} ({kind_j}) | Δkm={km:.1f}, ΔCO2={dco2:.2f}, load={load}, t={t_arr}")
                step += 1
            if kind_j in ("CUST_DEL","CUST_PU") or j == data["depot"]:
                poly.append(data["coords"][j])
            idx = nxt
        total_km += veh_km; total_co2 += veh_co2; routes_xy[v] = poly
        print(f"Vehicle {v:02d} summary | km={veh_km:.1f}, CO2={veh_co2:.2f}")
    print(f"\n🌍 Global: total_km={total_km:.1f}, total_CO2={total_co2:.2f}")

    # plot
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7,7))
    fig.patch.set_facecolor("white"); ax.set_facecolor("white")
    dx, dy = data["coords"][0]
    ax.scatter([dx],[dy],c="black",s=120,label="Depot 0"); ax.text(dx+1,dy+1,"0",color="black")
    # customers (show D/P & TW from base)
    for i, node in enumerate(data["cust_del_node"]):
        x,y = data["coords"][node]
        qd = data["deliver_q"][i]; qp = data["pickup_q"][i]
        e,l = data["time_windows"][node]
        tag = f"{i+1}\nD={qd}" + (f" P={qp}" if qp>0 else "") + f" [{int(e)},{int(l)}]"
        ax.scatter([x],[y],c="tab:blue",s=60,edgecolors="white",linewidths=2)
        ax.text(x+0.8,y+0.8,tag,fontsize=8,color="tab:blue",
                bbox=dict(boxstyle="round,pad=0.2",fc="white",alpha=0.85,lw=0))
    for v, poly in routes_xy.items():
        if len(poly)>=2:
            xs,ys = zip(*poly); ax.plot(xs,ys,linewidth=1.8,alpha=0.9,label=f"Veh#{v:02d}")
    ax.set_title(f"Single-Depot VRPSPD (Reuse VRPTW settings)\nTotal km={total_km:.1f}, Total CO2={total_co2:.2f}")
    ax.grid(True, linestyle=":", alpha=0.3); ax.set_aspect("equal", adjustable="datalim")
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, labels, loc="center left", bbox_to_anchor=(1.02,0.5), fontsize=8, frameon=True)
    fig.subplots_adjust(right=0.80)
    plt.tight_layout(); plt.savefig("single_depot_vrpspd.png", dpi=150, bbox_inches="tight")
    print("🖼 Saved: single_depot_vrpspd.png")

if __name__ == "__main__":
    # ←—— 关键：直接复用 VRPTW 的 tunables / 数据 ——→
    from single_depot_vrptw import create_data as create_vrptw_data
    base = create_vrptw_data()                      # 这里包含你 VRPTW 的所有前置设置
    EOL_RATIO = 0.4                                 # 仅新增一个“换机比例”参数
    data = build_vrpspd_from_vrptw(base, eol_ratio=EOL_RATIO,
                                   service_time_deliver=(base["service"][1] if "service" in base and len(base["service"])>1 else 3.0),
                                   service_time_pickup=(base["service"][1] if "service" in base and len(base["service"])>1 else 3.0))
    solve_and_plot(data)