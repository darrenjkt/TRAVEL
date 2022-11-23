import travel
import open3d as o3d
import numpy as np

min_range_ = 1.0
max_range_= 300.0  
tgf_res = 4.0
num_iter = 3
num_lpr = 2
num_min_pts =  2
th_seeds =  0.5
th_dist =  0.2
th_outlier = 1.0
th_normal = 0.97
th_weight = 100
th_obstacle = 0.5
th_lcc_normal_similarity = 0.125
th_lcc_planar_dist = 0.25
refine_mode=False
viz_mode=False

tgs = travel.TravelGroundSeg()
tgs.setParams(max_range_, min_range_, tgf_res, 
                              num_iter, num_lpr, num_min_pts, th_seeds, 
                              th_dist, th_outlier, th_normal, th_weight, 
                              th_lcc_normal_similarity, th_lcc_planar_dist, th_obstacle,
                              refine_mode, viz_mode)

# Read pcd
pcd = o3d.io.read_point_cloud('/TRAVEL/build/demo.pcd')
# in_pc = np.asarray(pcd.points)
ground_pc = o3d.utility.Vector3dVector()
nonground_pc = o3d.utility.Vector3dVector()
ground_seg_time = 0

print(f'Loaded input cloud: {np.asarray(pcd.points).shape}')
tgs.estimateGround(pcd.points, ground_pc, nonground_pc, ground_seg_time)
print(f'Ground cloud: {np.asarray(ground_pc).shape}')
print(f'Non-ground cloud: {np.asarray(nonground_pc).shape}')

ground_pcd = o3d.geometry.PointCloud()
ground_pcd.points = ground_pc
o3d.io.write_point_cloud('ground.pcd', ground_pcd)

nonground_pcd = o3d.geometry.PointCloud()
nonground_pcd.points = nonground_pc
o3d.io.write_point_cloud('nonground.pcd', nonground_pcd)
