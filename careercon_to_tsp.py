# careercon_to_tsp.py — run this once
import pandas as pd
import numpy as np

# Load raw Kaggle files (put them in the same folder)
X_train = pd.read_csv("X_train.csv")
X_test  = pd.read_csv("X_test.csv")
data = pd.concat([X_train, X_test], ignore_index=True)

def trajectory_to_xy(row):
    # Extract orientation (Euler angles) from 128 timesteps
    orient_x = row.filter(like='orientation_X').values
    orient_y = row.filter(like='orientation_Y').values
    # Extract angular velocity (used to integrate into a path)
    vel_x = row.filter(like='angular_velocity_X').values
    vel_y = row.filter(like='angular_velocity_Y').values

    dt = 0.02  # 50 Hz data
    # Simple integration → rough 2D path (this is what the robot actually moved)
    x = np.cumsum(vel_x) * dt
    y = np.cumsum(vel_y) * dt

    # Center the path so it doesn’t drift off the page
    x = x - x.mean()
    y = y - y.mean()

    return list(zip(x, y))

# Convert first 1000 trajectories (you can change to 3204 for all)
all_paths = []
for idx in range(1000):
    path = trajectory_to_xy(data.iloc[idx])
    all_paths.append(path)

# Save as your script expects: instance,x,y
with open("careercon_1000_real.csv", "w") as f:
    f.write("instance,x,y\n")
    for inst_id, path in enumerate(all_paths):
        for pt_id, (xx, yy) in enumerate(path):
            f.write(f"{inst_id},{xx:.6f},{yy:.6f}\n")

print("Done — careercon_1000_real.csv created (1000 instances × 128 points)")
print("Now run: python3 cooks_ruler_Euclidean.py careercon_1000_real.csv")
print(" OR:")
print("python3 cooks_ruler.py careercon_1000.csv")
print("This will probably work too but x,y in the data needs to be lat,lon")
