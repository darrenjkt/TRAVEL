import travel
import open3d as o3d
import numpy as np

class Travel:
  def __init__(self):
    self.tgs = travel.TravelGroundSeg()
    self.set_params()

  def set_params(self, 
              min_range=4.0,max_range=300.0,tgf_res=4.0,
              num_iter=3,num_lpr=2,num_min_pts=2,
              th_seeds=0.5,th_dist=0.2,th_outlier=1.0,th_normal=0.97,
              th_weight=100,th_obstacle=0.5,
              th_lcc_normal_similarity=0.125,th_lcc_planar_dist=0.25,
              refine_mode=False,viz_mode=False):

    self.tgs.setParams(max_range, min_range, tgf_res, 
                      num_iter, num_lpr, num_min_pts, th_seeds, 
                      th_dist, th_outlier, th_normal, th_weight, 
                      th_lcc_normal_similarity, th_lcc_planar_dist, th_obstacle,
                      refine_mode, viz_mode)

  def estimate_ground(self, points):
    """
    Takes in a numpy array of points (N,M) and returns the ground and nonground pc
    """
    in_pc = o3d.utility.Vector3dVector(points[:,:3])
    ground_pc = o3d.utility.Vector3dVector()
    nonground_pc = o3d.utility.Vector3dVector()
    ground_seg_time = 0
    self.tgs.estimateGround(in_pc, ground_pc, nonground_pc, ground_seg_time)
    
    return np.asarray(ground_pc), np.asarray(nonground_pc)
       

if __name__ == "__main__":
  # from travel_api import Travel # if using from another python file
  
  # Read pcd
  pcd = o3d.io.read_point_cloud('/TRAVEL/demo.pcd')
  in_pc = np.asarray(pcd.points)
  print(f'Loaded input cloud: {np.asarray(pcd.points).shape}')

  ground_seg = Travel()
  ground_pc, nonground_pc = ground_seg.estimate_ground(in_pc)

  print(f'Ground cloud: {ground_pc.shape}')
  print(f'Non-ground cloud: {nonground_pc.shape}')