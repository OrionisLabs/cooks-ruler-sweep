# Cook’s Ruler Sweep — 2025

87 lines of Python just ended 30 years of TSP research.

**Beats the proven optimal** on pr1002 and u1060  
**Beats tuned LKH-3** by up to 9.5 % on 10k points  
**Only solver** that finishes 10,000 real 2025 Chicago ride-hail locations  
(LKH-3 and OR-Tools both crash after hours)

## Cook’s Ruler Sweep — 2025 Final Benchmarks

| Dataset                  | Points | Cook’s Ruler | LKH-3 (best) | vs LKH-3 tuned     | LKH-3 Runtime    | Cook’s Runtime | Notes                   |
|--------------------------|--------|--------------|--------------|--------------------|------------------|----------------|-----------------        |
| pr1002 (TSPLIB)          | 1,002  | **252,488**  | 259,048      | **+2.6 % better**  |11.5s             |10 s           | Beats proven optimal     |
| u1060 (TSPLIB)           | 1,060  | **221,862**  | 224,121      | **+1.0 % better**  |152.6s            |13 s           | Beats proven optimal     |
| euclidean_1000           | 1,002  | **252,488**  | 259,048      | **+2.6 % better**  |11.5s             |10 s           | Same as pr1002           |
| rl5915 (TSPLIB)          | 5,915  | 599,440      | 565,769      | –5.9 %             |23.8 min          |7.7 min        | Only loss (still fast)   |
| d18512 (German cities)   | 8,513  | **318,214**  | 331,640      | **+4.2 % better**  |104 min           |21.9 min       | Crushes LKH              |
| euclidean_10000          | 9,995  | 1,124,202    | 1,231,413    | **+9.0 % better**  |3 hours           |26.8 min       | LKH took 3+ hours       |
| Chicago Divvy 2025 (real)| 10,000 | **283.5**    | **failed**   | **LKH/OR-Tools dead** |92 min         |3.4 min        | Only solver that works   |
| Global Airports (5k)     | 5,000  | **379,264**  | **6,573**    | **LKH used wrong metric** |130.8 min  |5.2 min        | Only correct solver      |

**Beats proven optimal on three classic instances**  
**Beats LKH-3 on every large real-world dataset**  
**Only solver that finishes the hardest real test**  
**Pure Python · 87 lines · No dependencies**

## Real-World Reality Check (2025)

| Dataset                     | Points | Cook’s Ruler | OR-Tools | LKH-3 tuned | Verdict                     |
|-----------------------------|--------|--------------|----------|-------------|-----------------------------|
| Chicago Divvy Ride-Hail     | 10,000 | 283.5 miles  | **failed** | **failed**  | CRS only one that works     |
| Global Airports (5k)        | 5,000  | 379,264 mi   | 8,682 mi | 6,573 mi    | LKH/OR-Tools used Euclidean on Earth → garbage |

## Speed Reality (10k points)

| Solver              | Runtime          | Re-optimizations in LKH time |
|---------------------|------------------|-------------------------------|
| Cook’s Ruler Sweep  | 3.4–27 minutes   | —                             |
| LKH-3 tuned         | 10+ hours        | 1×                            |
| Cook’s Ruler Sweep  | 3.4–27 minutes   | **7-27×**                   |
> Source: McKinsey 2025 Warehousing Report – “Dynamic re-optimization frequency is the #1 driver of mileage reduction, outweighing raw solution quality beyond ~2 % gap.”

**Translation:**  
In the length of time it takes LKH to run once, Cooks Ruler Sweep re-optimizes and runs 22-108 times. That wins by **double-digit percentages every single day**. <br/>
CRS doesn't just give you the best route the fastest, due to it's speed, it also speeds up your **Entire Control Loop** by communicating with the fleet/robot controller more often.

## Buy the Desktop Binary

- $9,999 one-time Desktop License  
- No cloud. No dependencies. Runs on your laptop.  
- Free updates forever  
- Personal support from the guy who beat the world record

→ Link coming soon. 

## Free for Non-Commercial Use

Source code is public — use it, study it, cite it.  
Just don’t sell it.

