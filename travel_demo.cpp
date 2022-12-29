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

#include "tgs.hpp"

struct PointXYZILID
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float    intensity;                 ///< laser intensity reading
  uint16_t label;                     ///< point label
  uint32_t id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZILID,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, label, label)
                                  (uint32_t, id, id))

using PointT = PointXYZILID;

boost::shared_ptr<travel::TravelGroundSeg<PointT>> travel_ground_seg;

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
  float min_range_ = 2.0;
  float max_range_= 300.0;  
  float tgf_res = 8.0;
  int num_iter = 3;
  int num_lpr = 5;
  int num_min_pts =  10;
  float th_seeds =  0.5;
  float th_dist =  0.125;
  float th_outlier = 1.0;
  float th_normal = 0.94;
  float th_weight = 100;
  float th_obstacle = 0.5;
  float th_lcc_normal_similarity = 0.03;
  float th_lcc_planar_dist = 0.2;

  bool refine_mode = false;
  bool viz_mode = false;

  travel_ground_seg->setParams(max_range_, min_range_, tgf_res, 
                              num_iter, num_lpr, num_min_pts, th_seeds, 
                              th_dist, th_outlier, th_normal, th_weight, 
                              th_lcc_normal_similarity, th_lcc_planar_dist, th_obstacle,
                              refine_mode, viz_mode);

  filtered_pc->points.reserve(cloud_in->points.size());
  for (auto &point : cloud_in->points){
      // bool is_nan = std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z);
      // double pt_range = 0.0;
      // if (is_nan){
      //     continue;
      // }    
      // pt_range = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
      // if (pt_range <= min_range_ || pt_range >= max_range_){
      //     continue;
      // }
      filtered_pc->push_back(point);
  }
  
  // Convert from pcl::PointCloud to std::vector<Eigen::Vector3d>
  std::vector<Eigen::Vector3d> nonground_pc_vec;
  std::vector<Eigen::Vector3d> ground_pc_vec;
  std::vector<Eigen::Vector3d> filtered_pc_vec;
  std::vector<Eigen::Vector3d> ground_inds_vec;
  for (auto &point : filtered_pc->points){
    Eigen::Vector3d point_eigen (static_cast<double>(point.x), static_cast<double>(point.y), static_cast<double>(point.z));  
    filtered_pc_vec.push_back(point_eigen);
  }

  // Apply traversable ground segmentation
  travel_ground_seg->estimateGround(filtered_pc_vec, ground_pc_vec, nonground_pc_vec, ground_inds_vec);

  // Convert from std::vector<Eigen::Vector3d> to pcl::PointCloud
  for (auto &point_vec : ground_pc_vec){
      PointT point;
      point.x = point_vec[0];
      point.y = point_vec[1];
      point.z = point_vec[2];
      ground_pc->push_back(point);
  }
  for (auto &point_vec : nonground_pc_vec){
      PointT point;
      point.x = point_vec[0];
      point.y = point_vec[1];
      point.z = point_vec[2];
      nonground_pc->push_back(point);
  }

  std::cout << "\033[1;35m Traversable-Ground Seg: " << filtered_pc->size() << " -> Ground: " << ground_pc->size() << ", NonGround: " << nonground_pc->size() << "\033[0m" << std::endl;

  // save floor cloud & nofloor cloud data.
  pcl::PCDWriter writer;
  ground_pc->width = 1;
  ground_pc->height = ground_pc->points.size();
  writer.write("onlyfloor.pcd", *ground_pc);

  nonground_pc->width = 1;
  nonground_pc->height = nonground_pc->points.size();
  writer.write("nofloor.pcd", *nonground_pc);

}
