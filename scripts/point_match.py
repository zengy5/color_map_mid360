import open3d as o3d
import numpy as np
import cv2
import color_cloud

#=======================================================
# 求点云质心
pcd = o3d.io.read_point_cloud("F:/data/visible_map.pcd")

points = np.asarray(pcd.points)
colors = np.asarray(pcd.colors)

color_threshold = 0.1 
valid_indices = np.sum(colors, axis=1) > color_threshold
points = points[valid_indices]
colors = colors[valid_indices]

color_diff_threshold = 0.1

blocks = {}
for i, color in enumerate(colors):
    found = False
    for key, centroid in blocks.items():
        color_diff = np.linalg.norm(color - np.array(key))
        if color_diff < color_diff_threshold:
            blocks[key].append(points[i])
            found = True
            break
    if not found:
        blocks[tuple(color)] = [points[i]]

points_3d = []
for key, block_points in blocks.items():
    if len(block_points) < 100:
        continue
    centroid = np.mean(block_points, axis=0)
    points_3d.append((centroid[0],centroid[1],centroid[2]))
    print(f"Color: {key}, Centroid: {centroid}")

centroid_cloud = o3d.geometry.PointCloud()
centroid_points = np.array(points_3d, dtype=np.float64)
centroid_cloud.points = o3d.utility.Vector3dVector(centroid_points)
set_color = [255, 0, 0]
set_colors = np.tile(set_color, (centroid_points.shape[0], 1))
centroid_cloud.colors = o3d.utility.Vector3dVector(set_colors/255.0)

o3d.io.write_point_cloud("data/result/colored_centroids.pcd", centroid_cloud)

#=========================================================================
# 将质心投影到图像中，并进行pnp求解
camera_intrinsic = np.float64([
    [918.608, 0.0, 631.377],
    [0.0, 915.973, 356.666],
    [0.0, 0.0, 1.0]
])
distCoeffs = np.float64([0.0, 0.0, 0.0, 0.0, 0.0])

lidar_to_camera_extrinsic = np.float64([
    [ 0.0, -1.0,  0.0, 0.0],
    [ 0.0,  0.0, -1.0, 0.0],
    [ 1.0,  0.0,  0.0, 0.0],
    [ 0, 0, 0, 1]
])
