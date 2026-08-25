import time
import numpy as np

# Simulate 30,000 points hitting a few walls
# Generate points heavily clustered
clusters = np.random.rand(10, 2) * 20.0
pts = []
for c in clusters:
    pts.append(c + np.random.randn(3000, 2) * 0.1)
points = np.vstack(pts)

t_start = time.time()
scaled = (points * 20.0).astype(np.int32)
# Using unique
unique_scaled, unique_indices = np.unique(scaled, axis=0, return_index=True)
unique_pts = points[unique_indices]
t_unique = time.time() - t_start

print(f"Time for np.unique: {t_unique*1000:.2f} ms. Original points: {len(points)}, Unique points: {len(unique_pts)}")

