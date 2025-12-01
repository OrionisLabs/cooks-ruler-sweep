# benchmark_all.py — FINAL, COMPLETE, NO MISSING ANYTHING
import pandas as pd
import numpy as np
import subprocess
import time
import math
import itertools
from pathlib import Path

# ----------------------- AUTO DISTANCE -----------------------
def get_distance_func(df):
    if {'lat', 'lon'}.issubset(df.columns):
        def dist(p1, p2):
            lat1, lon1 = p1
            lat2, lon2 = p2
            R = 3958.8
            dlat = math.radians(lat2 - lat1)
            dlon = math.radians(lon2 - lon1)
            a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            return R * c
        return dist
    else:
        return lambda p1, p2: np.linalg.norm(np.array(p1) - np.array(p2))

# ----------------------- SHARED 2-OPT -----------------------
def two_opt_tour(points, dist_func):
    n = len(points)
    tour = points[:]
    improved = True
    while improved:
        improved = False
        for i in range(1, n - 2):
            for j in range(i + 2, n):
                if j - i == 1: continue
                old = dist_func(tour[i-1], tour[i]) + dist_func(tour[j-1], tour[j])
                new = dist_func(tour[i-1], tour[j-1]) + dist_func(tour[i], tour[j])
                if new < old:
                    tour[i:j] = tour[i:j][::-1]
                    improved = True
    return tour

# ----------------------- COOK'S RULER SWEEP -----------------------
def cooks_ruler_sweep(points, df):
    dist_func = get_distance_func(df)
    n = len(points)
    labels = list(range(n))

    # Distance matrix
    dist_matrix = {(i, j): dist_func(points[i], points[j]) for i in labels for j in labels}

    # Farthest pair
    max_d = 0
    START = END = None
    for a, b in itertools.combinations(labels, 2):
        d = dist_matrix[(a, b)]
        if d > max_d:
            max_d, START, END = d, a, b

    # Projection
    total_line = dist_matrix[(START, END)]
    proj = {}
    for c in labels:
        d_sc = dist_matrix[(START, c)]
        d_ec = dist_matrix[(END, c)]
        if total_line == 0:
            proj[c] = 0
        else:
            cos_a = (d_sc**2 + total_line**2 - d_ec**2) / (2 * d_sc * total_line) if d_sc > 0 else 0
            cos_a = max(min(cos_a, 1), -1)
            proj[c] = d_sc * cos_a

    min_p = min(proj.values())
    max_p = max(proj.values())
    span = max_p - min_p if max_p > min_p else 1
    logical_x = lambda c: (proj[c] - min_p) / span

    # Ruler sweep
    ruler = 1.0 / n
    tour = [START]
    visited = {START}
    pos = 0.0
    while pos < 1.0:
        pos = min(pos + ruler, 1.0)
        candidates = [c for c in labels if c not in visited and abs(logical_x(c) - pos) <= ruler * 1.3]
        if candidates:
            next_c = min(candidates, key=lambda c: dist_matrix[(tour[-1], c)])
            tour.append(next_c)
            visited.add(next_c)

    if tour[-1] != END:
        tour.append(END)

    # 2-opt using SHARED function
    tour_points = [points[i] for i in tour]
    tour_points = two_opt_tour(tour_points, dist_func)

    # Final length
    length = 0.0
    for i in range(len(tour_points)):
        length += dist_func(tour_points[i], tour_points[(i + 1) % len(tour_points)])
    return length

# ----------------------- NN + 2-OPT -----------------------
def nn_plus_2opt(points, df):
    dist_func = get_distance_func(df)
    n = len(points)
    tour = [0]
    unvisited = set(range(1, n))
    while unvisited:
        next_c = min(unvisited, key=lambda i: dist_func(points[tour[-1]], points[i]))
        tour.append(next_c)
        unvisited.remove(next_c)

    tour_points = [points[i] for i in tour]
    tour_points = two_opt_tour(tour_points, dist_func)

    length = 0.0
    for i in range(len(tour_points)):
        length += dist_func(tour_points[i], tour_points[(i + 1) % len(tour_points)])
    return length

# ----------------------- OR-TOOLS -----------------------
def ortools_solve(points):
    try:
        from ortools.constraint_solver import routing_enums_pb2
        from ortools.constraint_solver import pywrapcp
        manager = pywrapcp.RoutingIndexManager(len(points), 1, 0)
        routing = pywrapcp.RoutingModel(manager)
        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return int(np.linalg.norm(points[from_node] - points[to_node]) * 1000)
        transit = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit)
        search_params = pywrapcp.DefaultRoutingSearchParameters()
        search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        search_params.time_limit.seconds = 30
        solution = routing.SolveWithParameters(search_params)
        return solution.ObjectiveValue() / 1000.0 if solution else None
    except:
        return None

# ----------------------- LKH-3 -----------------------
def lkh_solve(points, extra_params=None):
    with open("temp.tsp", "w") as f:
        f.write(f"NAME: temp\nTYPE: TSP\nDIMENSION: {len(points)}\nEDGE_WEIGHT_TYPE: EUC_2D\nNODE_COORD_SECTION\n")
        for i, (x, y) in enumerate(points, 1):
            f.write(f"{i} {x:.6f} {y:.6f}\n")
        f.write("EOF\n")
    with open("lkh.par", "w") as f:
        base = ["PROBLEM_FILE = temp.tsp", "RUNS = 10", "SEED = 42"]
        if extra_params:
            f.write("\n".join(base + extra_params) + "\n")
        else:
            f.write("\n".join(base) + "\n")
    start = time.time()
    result = subprocess.run(["./LKH", "lkh.par"], capture_output=True, text=True)
    runtime = time.time() - start
    length = None
    for line in result.stdout.splitlines():
        if "Length" in line or "Cost" in line:
            try:
                length = float(line.split()[-1])
                break
            except:
                pass
    return length, runtime

# ----------------------- MAIN -----------------------
def benchmark(csv_file):
    df = pd.read_csv(csv_file)
    if {'lat', 'lon'}.issubset(df.columns):
        points = df[['lat', 'lon']].values
    elif {'x', 'y'}.issubset(df.columns):
        points = df[['x', 'y']].values
    else:
        print("Need lat/lon or x/y columns")
        return
    points = points.astype(float)

    print(f"\n=== {Path(csv_file).stem} — {len(points)} points ===\n")

    results = {}

    # Cook's Ruler
    start = time.time()
    results["Cook's Ruler"] = (cooks_ruler_sweep(points, df), time.time() - start)

    # NN + 2-opt
    start = time.time()
    results["NN + 2-opt"] = (nn_plus_2opt(points, df), time.time() - start)

    # OR-Tools
    start = time.time()
    ort = ortools_solve(points)
    results["OR-Tools"] = (ort, time.time() - start) if ort else ("failed", time.time() - start)

    # LKH-3 default & tuned
    lkh_def, t_def = lkh_solve(points)
    results["LKH-3 default"] = (lkh_def, t_def)
    #use for less than 10k
    lkh_tuned, t_tuned = lkh_solve(points)
    #use for 10k
    #lkh_tuned, t_tuned = lkh_solve(points, ["MAX_CANDIDATES = 5", "KICKS = 4", "POPMUSIC_TRIALS = 100"])
    results["LKH-3 tuned"] = (lkh_tuned, t_tuned)

    # Print table
    print(f"{'Solver':<20} {'Length':>14} {'Runtime':>10} {'vs NN+2opt':>12}")
    print("-" * 60)
    baseline = results["NN + 2-opt"][0]
    for name, (length, rt) in results.items():
        if length and length != "failed":
            pct = (baseline - length) / baseline * 100 if baseline else 0
            print(f"{name:<20} {length:>14,.1f} {rt:>8.1f}s {pct:>+11.1f}%")
        else:
            print(f"{name:<20} {'failed':>14} {rt:>8.1f}s")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python3 benchmark_all.py <csv_file>")
    else:
        benchmark(sys.argv[1])