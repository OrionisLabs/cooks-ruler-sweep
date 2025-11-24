import math
import random
import pandas as pd
import argparse

def euclid(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def nearest_neighbor(points):
    n = len(points)
    unvisited = set(range(n))
    tour = [unvisited.pop()]
    while unvisited:
        current = tour[-1]
        next_city = min(unvisited, key=lambda i: euclid(points[current], points[i]))
        tour.append(next_city)
        unvisited.remove(next_city)
    return tour

def three_opt(tour_points):
    n = len(tour_points)
    improved = True
    iterations = 0
    while improved and iterations < 150:
        improved = False
        iterations += 1
        for i in range(n - 5):
            for j in range(i + 2, n - 3):
                a, b, c, d = tour_points[i], tour_points[i+1], tour_points[j], tour_points[j+1]
                if euclid(a, c) + euclid(b, d) < euclid(a, b) + euclid(c, d) - 1e-6:
                    tour_points[i+1:j+1] = tour_points[i+1:j+1][::-1]
                    improved = True
                    break
            if improved: break
    return tour_points

def cooks_ruler_seed(points, tolerance=1.3):
    n = len(points)
    if n < 2: return list(range(n))
    
    # FORCE diameter along warehouse aisles (X-axis)
    min_x = min(p[0] for p in points)
    max_x = max(p[0] for p in points)
    si = next(i for i, p in enumerate(points) if math.isclose(p[0], min_x))
    ei = next(i for i, p in enumerate(points) if math.isclose(p[0], max_x))
    
    S, E = points[si], points[ei]
    vec_SE = (E[0]-S[0], E[1]-S[1])
    D2 = vec_SE[0]**2 + vec_SE[1]**2 or 1
    proj = {}
    for i, pt in enumerate(points):
        vec_SP = (pt[0]-S[0], pt[1]-S[1])
        proj[i] = (vec_SP[0]*vec_SE[0] + vec_SP[1]*vec_SE[1]) / D2
    
    min_p = min(proj.values())
    span = max(proj.values()) - min_p or 1
    lx = lambda i: (proj[i] - min_p) / span
    
    step = 1.0 / n
    tour = [si]
    visited = {si}
    cur = si
    log_x = 0.0
    
    while log_x < 1.0:
        log_x = min(log_x + step, 1.0)
        cand = [i for i in range(n) if i not in visited and abs(lx(i) - log_x) <= step * tolerance]
        if cand:
            cand.sort(key=lambda i: euclid(points[cur], points[i]))
            for i in cand:
                tour.append(i)
                visited.add(i)
                cur = i
    
    if ei not in visited:
        tour.append(ei)
    
    while len(tour) < n:
        remaining = [i for i in range(n) if i not in tour]
        next_i = min(remaining, key=lambda i: euclid(points[cur], points[i]))
        tour.append(next_i)
        cur = next_i
    
    return tour

def ga_cooks_ruler(points):
    n = len(points)
    pop_size = 120
    generations = 200
    
    print("Starting ALL-IN GA — 120 individuals, 200 generations...")
    
    population = [cooks_ruler_seed(points, tolerance=1.3) for _ in range(pop_size)]
    
    for gen in range(generations):
        fitness = [sum(euclid(points[t[i]], points[t[(i+1)%n]]) for i in range(n)) for t in population]
        if gen % 40 == 0 or gen == generations - 1:
            print(f"  Gen {gen:3d}: {min(fitness):,.1f}")
        
        elites = [x for _, x in sorted(zip(fitness, population))[:pop_size//4]]
        offspring = elites.copy()
        
        while len(offspring) < pop_size:
            p1, p2 = random.sample(elites, 2)
            start, end = sorted(random.sample(range(n), 2))
            segment = p1[start:end]
            remaining = [x for x in p2 if x not in segment]
            child = remaining[:start] + segment + remaining[start:]
            if random.random() < 0.15:
                i, j = random.sample(range(n), 2)
                child[i], child[j] = child[j], child[i]
            offspring.append(child)
        population = offspring
    
    best_tour = min(population, key=lambda t: sum(euclid(points[t[i]], points[t[(i+1)%n]]) for i in range(n)))
    best_points = [points[i] for i in best_tour] + [points[best_tour[0]]]
    best_points = three_opt(best_points)
    final_len = sum(euclid(best_points[i], best_points[i+1]) for i in range(len(best_points)-1))
    return final_len

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('file')
    parser.add_argument('--name', default='Warehouse')
    args = parser.parse_args()
    
    df = pd.read_csv(args.file)
    points = [(row['x'], row['y']) for _, row in df.iterrows()]
    n = len(points)
    
    print(f"\n=== {args.name} — {n} points ===\n")
    
    nn_len = sum(euclid(points[nearest_neighbor(points)[i]], 
                       points[nearest_neighbor(points)[(i+1)%n]]) for i in range(n))
    print(f"Nearest Neighbor: {nn_len:,.1f}")
    
    final_len = ga_cooks_ruler(points)
    improvement = (nn_len - final_len) / nn_len * 100
    
    print(f"\nCooks Ruler ALL-IN: {final_len:,.1f}")
    print(f"Improvement: {improvement:+.1f}% vs Nearest Neighbor")
