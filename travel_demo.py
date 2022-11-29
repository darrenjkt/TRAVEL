from travel_api import Travel
import open3d as o3d
import numpy as np

# Read pcd
pcd = o3d.io.read_point_cloud('/TRAVEL/demo.pcd')
in_pc = np.asarray(pcd.points)
print(f'Loaded input cloud: {np.asarray(pcd.points).shape}')

ground_seg = Travel()
ground_pc, nonground_pc, ground_inds = ground_seg.estimate_ground(in_pc)

print(f'Ground cloud: {ground_pc.shape}')
print(f'Non-ground cloud: {nonground_pc.shape}')

# Visualize
ground_pcd = o3d.geometry.PointCloud()
ground_pcd.points = o3d.utility.Vector3dVector(ground_pc)
ground_pcd.paint_uniform_color([0,0,0])
nonground_pcd = o3d.geometry.PointCloud()
nonground_pcd.points = o3d.utility.Vector3dVector(nonground_pc)

o3d.visualization.draw_geometries([ground_pcd, nonground_pcd])
