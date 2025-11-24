import math
import itertools
import pandas as pd
import argparse
import os

def haversine(p1, p2):
    lat1, lon1 = p1
    lat2, lon2 = p2
    R = 3958.8
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

dist = haversine

# 2-opt
def two_opt(points):
    n = len(points)
    improved = True
    while improved:
        improved = False
        for i in range(1, n-2):
            for j in range(i+2, n):
                if j - i == 1: continue
                if (dist(points[i-1], points[i]) + dist(points[j-1], points[j]) >
                    dist(points[i-1], points[j-1]) + dist(points[i], points[j])):
                    points[i:j] = points[i:j][::-1]
                    improved = True
    return points

def run_cooks_ruler(points_dict, dataset_name):
    labels = list(points_dict.keys())
    n = len(labels)
    
    print(f"\n=== {dataset_name} ({n} points) ===")
    
    # Pre-compute distance matrix
    dist_matrix = {}
    for a in labels:
        for b in labels:
            dist_matrix[(a, b)] = dist(points_dict[a], points_dict[b])
    
    # Farthest pair
    max_d = 0
    START = END = None
    for a, b in itertools.combinations(labels, 2):
        d = dist_matrix[(a, b)]
        if d > max_d:
            max_d, START, END = d, a, b
    
    print(f"Start: {START}, End: {END}, Distance: {max_d:,.1f} units")
    
    # === Cosine-law projection =====
    proj_dist = {}
    total_line = dist_matrix[(START, END)]
    for city in labels:
        d_sc = dist_matrix[(START, city)]
        d_ec = dist_matrix[(END, city)]
        if total_line == 0:
            proj_dist[city] = 0
        else:
            cos_angle = (d_sc**2 + total_line**2 - d_ec**2) / (2 * d_sc * total_line) if d_sc > 0 else 0
            cos_angle = max(min(cos_angle, 1), -1)
            proj_dist[city] = d_sc * cos_angle
    
    min_proj = min(proj_dist.values())
    max_proj = max(proj_dist.values())
    span = max_proj - min_proj if max_proj > min_proj else 1
    def logical_x(city): return (proj_dist[city] - min_proj) / span
    
    # === Cook's Ruler ===
    ruler = 1.0 / n
    tour = [START]
    visited = {START}
    log_x = 0.0
    
    while log_x < 1.0:
        log_x = min(log_x + ruler, 1.0)
        candidates = [(c, points_dict[c]) for c in labels 
                     if c not in visited and abs(logical_x(c) - log_x) <= ruler * 1.3]
        if candidates:
            candidates.sort(key=lambda x: dist_matrix[(tour[-1], x[0])])
            for c, pt in candidates:
                tour.append(c)
                visited.add(c)
    
    if tour[-1] != END:
        tour.append(END)
    
    # Tour length
    lrs_path = [points_dict[c] for c in tour] + [points_dict[tour[0]]]
    lrs_length = sum(dist(lrs_path[i], lrs_path[i+1]) for i in range(len(lrs_path)-1))
    
    # === Nearest Neighbor ====
    nn_tour = [START]
    current = START
    unvisited = set(labels) - {START}
    while unvisited:
        next_c = min(unvisited, key=lambda c: dist_matrix[(current, c)])
        nn_tour.append(next_c)
        current = next_c
        unvisited.remove(next_c)
    if nn_tour[-1] != END:
        nn_tour.append(END)
    nn_path = [points_dict[c] for c in nn_tour] + [points_dict[nn_tour[0]]]  # ← FIXED LINE
    nn_length = sum(dist(nn_path[i], nn_path[i+1]) for i in range(len(nn_path)-1))
    
    # 2-opt
    opt_path = two_opt(lrs_path[:-1]) + [lrs_path[0]]
    opt_length = sum(dist(opt_path[i], opt_path[i+1]) for i in range(len(opt_path)-1))
    
    # Print
    print(f"{'Algorithm':<22} {'Length (units)':<15} {'vs NN'}")
    print("-" * 55)
    print(f"{'Nearest Neighbor':<22} {nn_length:>12,.1f} {'-':>8}")
    #print(f"{'Cooks Ruler (raw)':<22} {lrs_length:>12,.1f} {((nn_length - lrs_length)/nn_length*100):+6.1f}%")
    print(f"{'Cooks Ruler':<22} {opt_length:>12,.1f} {((nn_length - opt_length)/nn_length*100):+6.1f}%")

# === CLI + File Loading ===
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Cook's Ruler Sweep — Earth version")
    parser.add_argument('file', help='Your CSV file (must have lat/lon columns)')
    parser.add_argument('--name', default='Your Data', help='Name to display (default: "Your Data")')
    args = parser.parse_args()
    
    if not os.path.exists(args.file):
        print(f"File not found: {args.file}")
        exit(1)
    
    df = pd.read_csv(args.file)
    if not {'lat', 'lon'}.issubset(df.columns):
        print("CSV must have 'lat' and 'lon' columns")
        exit(1)
    
    points = {f"p{i}": (row['lat'], row['lon']) for i, row in df.iterrows()}
    run_cooks_ruler(points, args.name)
