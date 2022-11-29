from travel_api import Travel
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# Read pcd
pcd = o3d.io.read_point_cloud('/TRAVEL/demo.pcd')
in_pc = np.asarray(pcd.points)
print(f'Loaded input cloud: {np.asarray(pcd.points).shape}')

ground_seg = Travel()
ground_pc, nonground_pc, ground_inds = ground_seg.estimate_ground(in_pc)

print(f'Ground cloud: {ground_pc.shape}')
print(f'Non-ground cloud: {nonground_pc.shape}')

# Visualize
nonground_o3d = o3d.geometry.PointCloud()
nonground_o3d.points = o3d.utility.Vector3dVector(nonground_pc)
cmap = plt.get_cmap('tab20')

# Db scan to cluster objects - downsample then cluster
downpcd, idx_c, idx_vox = nonground_o3d.voxel_down_sample_and_trace(0.2, nonground_o3d.get_min_bound(), nonground_o3d.get_max_bound(), False)
cluster_labels = np.array(downpcd.cluster_dbscan(eps=0.3, min_points=2))

# Apply cluster ID to every point in the voxel
pcd_cids = np.zeros((np.asarray(nonground_o3d.points).shape[0]), dtype=np.int32)
for row_id in range(len(cluster_labels)):
  pcd_cids[idx_vox[row_id]] = cluster_labels[row_id]

colors = np.ones((pcd_cids.shape[0],3))
colors[pcd_cids != -1] = cmap(pcd_cids[pcd_cids != -1]%20)[:,:3]
nonground_o3d.colors = o3d.utility.Vector3dVector(colors)

ground_pcd = o3d.geometry.PointCloud()
ground_pcd.points = o3d.utility.Vector3dVector(ground_pc)
ground_pcd.paint_uniform_color([0,0,0])
o3d.visualization.draw_geometries([ground_pcd, nonground_o3d])
