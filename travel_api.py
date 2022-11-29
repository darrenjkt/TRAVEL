import travel
import open3d as o3d
import numpy as np

class Travel:
  def __init__(self):
    self.tgs = travel.TravelGroundSeg()
    self.set_params()

  def set_params(self, 
              min_range=2.0,max_range=75.0,tgf_res=8.0,
              num_iter=3,num_lpr=5,num_min_pts=5,
              th_seeds=0.5,th_dist=0.125,th_outlier=1.0,th_normal=0.94,
              th_weight=100,th_obstacle=0.5,
              th_lcc_normal_similarity=0.03,th_lcc_planar_dist=0.2,
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
    # Converting np array to np.float64 speeds up conversion to Vector3dVector by a lot (0.227s -> 0.003s)
    # https://github.com/isl-org/Open3D/issues/1045#issuecomment-513169038
    in_pc = o3d.utility.Vector3dVector(points[:,:3].astype(dtype=np.float64))
    ground_pc = o3d.utility.Vector3dVector()
    nonground_pc = o3d.utility.Vector3dVector()
    ground_inds = o3d.utility.Vector3dVector()
    self.tgs.estimateGround(in_pc, ground_pc, nonground_pc, ground_inds)

    return np.asarray(ground_pc, dtype=np.float64), np.asarray(nonground_pc, dtype=np.float64), np.asarray(ground_inds, dtype=int)[:,0]
       
if __name__ == "__main__":
  # from travel_api import Travel # if using from another python file
  
  # Read pcd
  pcd = o3d.io.read_point_cloud('/TRAVEL/demo.pcd')
  in_pc = np.asarray(pcd.points)
  print(f'Loaded input cloud: {np.asarray(pcd.points).shape}')

  ground_seg = Travel()
  ground_pc, nonground_pc, ground_inds = ground_seg.estimate_ground(in_pc)

  print(f'Ground cloud: {ground_pc.shape}')
  print(f'Non-ground cloud: {nonground_pc.shape}')