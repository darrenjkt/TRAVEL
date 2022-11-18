#define PCL_NO_PRECOMPILE

// Might need to: apt install libpcl-dev

#include <iostream>
#include <fstream>
#include <memory>
#include <chrono>
#include <map>

#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

//#include <opencv/cv.h>
#include <signal.h>

#include "aos.hpp"
#include "tgs.hpp"

struct PointXYZILID
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float    intensity;                 ///< laser intensity reading
  uint16_t label;                     ///< point label
  uint16_t id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZILID,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, label, label)
                                  (uint16_t, id, id))

using PointT = PointXYZILID;

boost::shared_ptr<travel::TravelGroundSeg<PointT>> travel_ground_seg;
boost::shared_ptr<travel::ObjectCluster<PointT>> travel_object_seg;

pcl::PointCloud<PointT>::Ptr cloud_in;
pcl::PointCloud<PointT>::Ptr filtered_pc;
pcl::PointCloud<PointT>::Ptr ground_pc;
pcl::PointCloud<PointT>::Ptr nonground_pc;
pcl::PointCloud<PointT>::Ptr outlier_pc;
pcl::PointCloud<PointT>::Ptr labeled_pc;

int main(void)
{	
	cloud_in.reset(new pcl::PointCloud<PointT>());
  filtered_pc.reset(new pcl::PointCloud<PointT>());
  ground_pc.reset(new pcl::PointCloud<PointT>());
  nonground_pc.reset(new pcl::PointCloud<PointT>());
  labeled_pc.reset(new pcl::PointCloud<PointT>());
  pcl::PCDReader reader;
	reader.read("demo.pcd", *cloud_in);
	std::cout << "Read Cloud Data Points Size: " << cloud_in->points.size() << std::endl;

  travel_ground_seg.reset(new travel::TravelGroundSeg<PointT>());

  // Ground seg
  float min_range_ = 1.0;
  float max_range_= 300.0;  
  float tgf_res = 4.0;
  int num_iter = 3;
  int num_lpr = 2;
  int num_min_pts =  2;
  float th_seeds =  0.5;
  float th_dist =  0.2;
  float th_outlier = 1.0;
  float th_normal = 0.97;
  float th_weight = 100;
  float th_obstacle = 0.5;
  float th_lcc_normal_similarity = 0.125;
  float th_lcc_planar_dist = 0.25;

  // Object clustering
  // int vert_scan = 64;
  // int horz_scan = 1800;
  // float min_vert_angle = 50.0;
  // float  max_vert_angle = 50.0;
  // int  downsample = 2;
  // float car_width = 1.0;
  // float car_length = 1.0;
  // float lidar_width_offset = 0.0;
  // float lidar_length_offset = 0.0;
  // float horz_merge_thres = 0.3;
  // float vert_merge_thres = 1.0;
  // int vert_scan_size = 4;
  // int horz_scan_size = 4;
  // int horz_skip_size = 4;
  // int horz_extension_size = 3;
  // int min_cluster_size = 4;
  // int max_cluster_size = 100;
  bool refine_mode = false;
  bool viz_mode = false;

  std::cout << "Max Range: " << max_range_ << std::endl;
  std::cout << "Min Range: " << min_range_ << std::endl;
  travel_ground_seg->setParams(max_range_, min_range_, tgf_res, 
                              num_iter, num_lpr, num_min_pts, th_seeds, 
                              th_dist, th_outlier, th_normal, th_weight, 
                              th_lcc_normal_similarity, th_lcc_planar_dist, th_obstacle,
                              refine_mode, viz_mode);

  filtered_pc->points.reserve(cloud_in->points.size());
  for (auto &point : cloud_in->points){
      bool is_nan = std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z);
      double pt_range = 0.0;
      if (is_nan){
          continue;
      }    
      pt_range = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
      if (pt_range <= min_range_ || pt_range >= max_range_){
          continue;
      }
      filtered_pc->push_back(point);
  }
  // Apply traversable ground segmentation
  double ground_seg_time = 0.0;
  travel_ground_seg->estimateGround(*filtered_pc, *ground_pc, *nonground_pc, ground_seg_time);
  std::cout << "\033[1;35m Traversable-Ground Seg: " << filtered_pc->size() << " -> Ground: " << ground_pc->size() << ", NonGround: " << nonground_pc->size() << "\033[0m" << std::endl;
  std::cout << "Traversable-Ground Seg time: " << ground_seg_time << std::endl;

  // save floor cloud & nofloor cloud data.
  pcl::PCDWriter writer;
  ground_pc->width = 1;
  ground_pc->height = ground_pc->points.size();
  writer.write("onlyfloor.pcd", *ground_pc);

  nonground_pc->width = 1;
  nonground_pc->height = nonground_pc->points.size();
  writer.write("nofloor.pcd", *nonground_pc);

}