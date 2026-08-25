import time
import numpy as np

# Simulate 30,000 points
points = np.random.rand(30000, 2) * 50.0

voxel_map = set()
accumulated_points = []

t_start = time.time()
pts_list = points.tolist()
for wx, wy in pts_list:
    grid_key = (int(wx * 20.0), int(wy * 20.0))
    if grid_key not in voxel_map:
        voxel_map.add(grid_key)
        accumulated_points.append((wx, wy))

print(f"Python loop time: {(time.time() - t_start)*1000:.2f} ms")

# Now Numpy way
t_start = time.time()
# multiply by 20 and cast to int
scaled = (points * 20.0).astype(np.int32)
# find unique voxels
_, unique_indices = np.unique(scaled, axis=0, return_index=True)
unique_pts = points[unique_indices]
# wait, we still need to check against the GLOBAL voxel_map, which is a set.
# checking a set in python might still be slow?
new_pts = []
for wx, wy in unique_pts.tolist():
    grid_key = (int(wx * 20.0), int(wy * 20.0))
    if grid_key not in voxel_map:
        voxel_map.add(grid_key)
        accumulated_points.append((wx, wy))
        
print(f"Numpy unique + loop time: {(time.time() - t_start)*1000:.2f} ms")

