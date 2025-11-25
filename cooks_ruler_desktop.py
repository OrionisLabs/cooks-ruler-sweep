# Cook's Ruler Sweep — Desktop Edition — $1,999 one-time
# Orionis Labs LLC — 2025

import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext
import numpy as np
import pandas as pd
import math
import os
import time

# --------------------- CORE SOLVER ---------------------
def euclid(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def cooks_ruler_seed(points, tolerance=1.3):
    n = len(points)
    if n < 2: return list(range(n))
    
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

def two_opt_fast(points_list):
    n = len(points_list)
    improved = True
    while improved:
        improved = False
        for i in range(1, n-2):
            for j in range(i+2, n):
                if j - i == 1: continue
                if (euclid(points_list[i-1], points_list[i]) + 
                    euclid(points_list[j-1], points_list[j]) >
                    euclid(points_list[i-1], points_list[j-1]) + 
                    euclid(points_list[i], points_list[j])):
                    points_list[i:j] = points_list[i:j][::-1]
                    improved = True
        if not improved: break
    return points_list

def final_solve(points):
    tour = cooks_ruler_seed(points, tolerance=1.3)
    tour_points = [points[i] for i in tour]
    tour_points = two_opt_fast(tour_points)

    final_tour = []
    for p in tour_points:
        # Find closest matching index (NumPy-safe)
        distances = np.sqrt(np.sum((points - p)**2, axis=1))
        idx = np.argmin(distances)
        final_tour.append(idx)

    length = sum(euclid(points[final_tour[i]], points[final_tour[(i+1)%len(final_tour)]]) 
                 for i in range(len(final_tour)))
    return length, final_tour

# --------------------- CSV LOADER ---------------------
def load_points_from_csv(filepath):
    df = pd.read_csv(filepath)
    x_cols = [c for c in df.columns if c.lower() in ['x','lon','longitude','long','lng']]
    y_cols = [c for c in df.columns if c.lower() in ['y','lat','latitude']]
    if x_cols and y_cols:
        return np.array(df[[x_cols[0], y_cols[0]]], dtype=float)
    numeric = df.select_dtypes(include=[np.number])
    if numeric.shape[1] >= 2:
        return numeric.iloc[:, :2].values
    raise ValueError("No X/Y coordinates found")

# --------------------- GUI ---------------------
root = tk.Tk()
root.title("Cook's Ruler Sweep — Desktop Edition")
root.configure(bg="#0d1117")

# MAXIMIZED (taskbar visible) — works on Windows, Mac, Linux
root.update_idletasks()
if os.name == "nt":  # Windows
    root.state('zoomed')
else:
    root.attributes('-zoomed', True)
root.geometry(f"{root.winfo_screenwidth()-100}x{root.winfo_screenheight()-100}+50+50")

# Title
tk.Label(root, text="Cook's Ruler Sweep", font=("Consolas", 28, "bold"),
         fg="#58a6ff", bg="#0d1117").pack(pady=30)

# File picker
path_frame = tk.Frame(root, bg="#0d1117")
path_frame.pack(pady=10)
file_var = tk.StringVar()
tk.Entry(path_frame, textvariable=file_var, width=90, font=("Consolas", 11)).pack(side="left", padx=10)
tk.Button(path_frame, text="Browse CSV", command=lambda: file_var.set(filedialog.askopenfilename(
    filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")])),
    bg="#238636", fg="white").pack(side="left")

# Solve
def solve():
    path = file_var.get()
    if not path or not os.path.exists(path):
        messagebox.showerror("Error", "Please select a valid CSV file")
        return
    try:
        points = load_points_from_csv(path)
        start_time = time.time()
        output.delete(1.0, tk.END)
        output.insert(tk.END, f"Loaded {len(points)} points — solving...\n\n")
        root.update()
        
        length, tour = final_solve(points)
        
        elapsed = time.time() - start_time
        result = f"TOUR LENGTH: {length:,.2f} units\n"
        result += f"NODES: {len(tour)}\n"
        result += f"RUNTIME: {elapsed:.2f} seconds\n\n"
        result += "OPTIMIZED TOUR ORDER:\n"
        for i, idx in enumerate(tour):
            result += f"{idx}"
            if i < len(tour)-1:
                result += " → "
            if (i + 1) % 15 == 0:
                result += "\n"
        result += "\n"
        output.insert(tk.END, result)
        
        global last_tour
        last_tour = tour
        
    except Exception as e:
        messagebox.showerror("Error", str(e))

tk.Button(root, text="SOLVE", command=solve, font=("Consolas", 18, "bold"),
          bg="#238636", fg="white", height=2, width=20).pack(pady=30)

# Output
output = scrolledtext.ScrolledText(root, bg="#161b22", fg="#f0f6fc",
                                   font=("Consolas", 11), wrap="word")
output.pack(padx=40, pady=10, fill="both", expand=True)

# Export
last_tour = []
def export_csv():
    global last_tour
    if not last_tour:
        messagebox.showinfo("Info", "Solve a problem first")
        return
    path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")])
    if path:
        header = "city_index"
        np.savetxt(path, last_tour, delimiter=",", fmt="%d", header=header, comments="")
        messagebox.showinfo("Success", f"Clean tour exported!\n{path}")

tk.Button(root, text="Export Tour CSV", command=export_csv,
          bg="#30363d", fg="#f0f6fc", font=("Consolas", 12)).pack(pady=10)

# Footer
tk.Label(root, text="Orionis Labs LLC — 2025", fg="#8b949e", bg="#0d1117",
         font=("Consolas", 10)).pack(side="bottom", pady=15)

root.mainloop()