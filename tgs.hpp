#ifndef TRAVEL_GSEG_H
#define TRAVEL_GSEG_H
//
// Created by Minho Oh & Euigon Jung on 1/31/22.
// We really appreciate Hyungtae Lim and Prof. Hyun Myung! :)
//
#include <iostream>
#include <vector>
#include <random>
#include <mutex>
#include <thread>
#include <chrono>
#include <math.h>
#include <queue>

#include <Eigen/Dense>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
// #include <opencv2/opencv.hpp>

namespace travel {

    #define PTCLOUD_SIZE 132000
    #define NODEWISE_PTCLOUDSIZE 5000

    #define UNKNOWN 1
    #define NONGROUND 2
    #define GROUND 3

    using Eigen::MatrixXf;
    using Eigen::JacobiSVD;
    using Eigen::VectorXf;

    template <typename PointType>
    bool point_z_cmp(PointType a, PointType b){
        return a.z<b.z;
    }

    struct TriGridIdx {
        int row, col, tri;
    };

    struct TriGridEdge {
        std::pair<TriGridIdx, TriGridIdx> Pair;
        bool is_traversable;
    };

    template <typename PointType>
    struct TriGridNode {
        int node_type;
        pcl::PointCloud<PointType> ptCloud;

        bool is_curr_data;
        
        // planar model
        Eigen::Vector3f normal;
        Eigen::Vector3f mean_pt;
        double d;

        Eigen::Vector3f singular_values;
        Eigen::Matrix3f eigen_vectors;
        double weight;

        double th_dist_d;
        double th_outlier_d;

        // graph_searching
        bool need_recheck;
        bool is_visited;
        bool is_rejection;
        int check_life;
        int depth;
    };

    struct TriGridCorner {
        double x, y;
        std::vector<double> zs;
        std::vector<double> weights;
    };

    template <typename PointType>
    using GridNode = std::vector<TriGridNode<PointType>>;

    template <typename PointType>
    using TriGridField = std::vector<std::vector<GridNode<PointType>>>;

    template <typename PointType>
    class TravelGroundSeg{
    private:

        bool REFINE_MODE_;
        bool VIZ_MDOE_;
        
        double MAX_RANGE_;
        double MIN_RANGE_;
        double TGF_RESOLUTION_;

        int NUM_ITER_;
        int NUM_LRP_;
        int NUM_MIN_POINTS_;
        
        double TH_SEEDS_;
        double TH_DIST_;
        double TH_OUTLIER_;
        
        double TH_NORMAL_;
        double TH_WEIGHT_;
        double TH_LCC_NORMAL_SIMILARITY_;
        double TH_LCC_PLANAR_MODEL_DIST_;
        double TH_OBSTACLE_HEIGHT_;

        TriGridField<PointType> trigrid_field_;
        std::vector<TriGridEdge> trigrid_edges_;
        std::vector<std::vector<TriGridCorner>> trigrid_corners_;
        std::vector<std::vector<TriGridCorner>> trigrid_centers_;

        pcl::PointCloud<PointType> empty_cloud_;
        TriGridNode<PointType>  empty_trigrid_node_;
        GridNode<PointType> empty_grid_nodes_;
        TriGridCorner empty_trigrid_corner_;
        TriGridCorner empty_trigrid_center_;

        pcl::PointCloud<PointType> ptCloud_tgfwise_ground_;
        pcl::PointCloud<PointType> ptCloud_tgfwise_nonground_;
        pcl::PointCloud<PointType> ptCloud_tgfwise_outliers_;
        pcl::PointCloud<PointType> ptCloud_tgfwise_obstacle_;
        pcl::PointCloud<PointType> ptCloud_nodewise_ground_;
        pcl::PointCloud<PointType> ptCloud_nodewise_nonground_;
        pcl::PointCloud<PointType> ptCloud_nodewise_outliers_;
        pcl::PointCloud<PointType> ptCloud_nodewise_obstacle_;

        pcl::PointCloud<PointType> ptCloud_nodewise_nonground_tmp;
        pcl::PointCloud<PointType> ptCloud_nodewise_ground_tmp;
        pcl::PointCloud<PointType> ptCloud_nodewise_obstacle_tmp;

    public:
        TravelGroundSeg(){

        };

        void setParams(const double max_range, const double min_range, const double resolution, 
                            const int num_iter, const int num_lpr, const int num_min_pts, const double th_seeds, 
                            const double th_dist, const double th_outlier, const double th_normal, const double th_weight, 
                            const double th_lcc_normal_similiarity, const double th_lcc_planar_model_dist, const double th_obstacle,
                            const bool refine_mode, const bool visualization) {

            MAX_RANGE_ = max_range;

            MIN_RANGE_ = min_range;
            
            TGF_RESOLUTION_ = resolution;
            
            NUM_ITER_ = num_iter;
            
            NUM_LRP_ = num_lpr;
            
            NUM_MIN_POINTS_ = num_min_pts;
            
            TH_SEEDS_ = th_seeds;
            
            TH_DIST_ = th_dist;
            
            TH_OUTLIER_ = th_outlier;

            TH_NORMAL_ = th_normal;

            TH_WEIGHT_ = th_weight;

            TH_LCC_NORMAL_SIMILARITY_ = th_lcc_normal_similiarity;

            TH_LCC_PLANAR_MODEL_DIST_ = th_lcc_planar_model_dist;

            TH_OBSTACLE_HEIGHT_ = th_obstacle;
            
            REFINE_MODE_ = refine_mode;
            VIZ_MDOE_ = visualization;

            initTriGridField(trigrid_field_);
            initTriGridCorners(trigrid_corners_, trigrid_centers_);

            ptCloud_tgfwise_ground_.clear();
            ptCloud_tgfwise_ground_.reserve(PTCLOUD_SIZE);
            ptCloud_tgfwise_nonground_.clear();
            ptCloud_tgfwise_nonground_.reserve(PTCLOUD_SIZE);
            ptCloud_tgfwise_outliers_.clear();
            ptCloud_tgfwise_outliers_.reserve(PTCLOUD_SIZE);
            ptCloud_tgfwise_obstacle_.clear();
            ptCloud_tgfwise_obstacle_.reserve(PTCLOUD_SIZE);
    
            ptCloud_nodewise_ground_.clear();
            ptCloud_nodewise_ground_.reserve(NODEWISE_PTCLOUDSIZE);
            ptCloud_nodewise_nonground_.clear();
            ptCloud_nodewise_nonground_.reserve(NODEWISE_PTCLOUDSIZE);
            ptCloud_nodewise_outliers_.clear();
            ptCloud_nodewise_outliers_.reserve(NODEWISE_PTCLOUDSIZE);
            ptCloud_nodewise_obstacle_.clear();
            ptCloud_nodewise_obstacle_.reserve(NODEWISE_PTCLOUDSIZE);

            ptCloud_nodewise_nonground_tmp.reserve(NODEWISE_PTCLOUDSIZE);
            ptCloud_nodewise_ground_tmp.reserve(NODEWISE_PTCLOUDSIZE);
            ptCloud_nodewise_obstacle_tmp.reserve(NODEWISE_PTCLOUDSIZE);
        };

        // Converted inputs to Eigen::Vector3d so that we can feed Open3D vectors in
        void estimateGround(const std::vector<Eigen::Vector3d>& cloud_in_vec,
                            std::vector<Eigen::Vector3d>& cloudGround_out_vec,
                            std::vector<Eigen::Vector3d>& cloudNonground_out_vec,
                            std::vector<Eigen::Vector3d>& ground_inds_vec){            
        
            // 0. Init
            pcl::PointCloud<PointType> cloud_in;
            pcl::PointCloud<PointType> cloudGround_out;
            pcl::PointCloud<PointType> cloudNonground_out;

            // Convert Eigen to PointCloud
            for (uint32_t i=0; i < cloud_in_vec.size(); i++) {
                PointType point;
                point.x = cloud_in_vec[i][0];
                point.y = cloud_in_vec[i][1];
                point.z = cloud_in_vec[i][2];
                point.id = i;                
                cloud_in.push_back(point);
               
            }
            ptCloud_tgfwise_outliers_.clear();
            ptCloud_tgfwise_outliers_.reserve(cloud_in.size());

            // 1. Embed PointCloud to TriGridField
            clearTriGridField(trigrid_field_);
            clearTriGridCorners(trigrid_corners_, trigrid_centers_);

            embedCloudToTriGridField(cloud_in, trigrid_field_);

            // 2. Node-wise Terrain Modeling
            modelNodeWiseTerrain(trigrid_field_);
            
            // 3. Breadth-first Traversable Graph Search
            BreadthFirstTraversableGraphSearch(trigrid_field_);
            setTGFCornersCenters(trigrid_field_, trigrid_corners_, trigrid_centers_);

            // 4. TGF-wise Traversable Terrain Model Fitting
            if (REFINE_MODE_){
                fitTGFWiseTraversableTerrainModel(trigrid_field_, trigrid_corners_, trigrid_centers_);
            }
            
            // 5. Ground Segmentation
            segmentTGFGround(trigrid_field_, ptCloud_tgfwise_ground_, ptCloud_tgfwise_nonground_, ptCloud_tgfwise_obstacle_, ptCloud_tgfwise_outliers_);
            cloudGround_out = ptCloud_tgfwise_ground_;
            cloudNonground_out = ptCloud_tgfwise_nonground_;            

            // Convert PointCloud back to Eigen::Vector3d            
            for (auto &point : cloudGround_out.points){
                Eigen::Vector3d point_eigen (static_cast<double>(point.x), static_cast<double>(point.y), static_cast<double>(point.z));  
                cloudGround_out_vec.push_back(point_eigen);

                Eigen::Vector3d inds (static_cast<double>(point.id), 0.0, 0.0);  
                ground_inds_vec.push_back(inds);
            }
            for (auto &point : cloudNonground_out.points){
                Eigen::Vector3d point_eigen (static_cast<double>(point.x), static_cast<double>(point.y), static_cast<double>(point.z));  
                cloudNonground_out_vec.push_back(point_eigen);
            }

            return;
        };

        TriGridIdx getTriGridIdx(const float& x_in, const float& y_in){
            TriGridIdx tgf_idx;
            int r_i = (x_in - tgf_min_x)/TGF_RESOLUTION_;
            int c_i = (y_in - tgf_min_y)/TGF_RESOLUTION_;
            int t_i = 0;
            double angle = atan2(y_in-(c_i*TGF_RESOLUTION_ + TGF_RESOLUTION_/2 + tgf_min_y), x_in-(r_i*TGF_RESOLUTION_ + TGF_RESOLUTION_/2 + tgf_min_x));

            if (angle>=(M_PI/4) && angle <(3*M_PI/4)){
                t_i = 1;
            } else if (angle>=(-M_PI/4) && angle <(M_PI/4)){
                t_i = 0;
            } else if (angle>=(-3*M_PI/4) && angle <(-M_PI/4)){
                t_i = 3;
            } else{
                t_i = 2;
            }
            tgf_idx.row = r_i;
            tgf_idx.col = c_i;
            tgf_idx.tri = t_i;
            return tgf_idx;
        }

        TriGridNode<PointType> getTriGridNode(const float& x_in, const float& y_in){
            TriGridNode<PointType> node;
            TriGridIdx node_idx = getTriGridIdx(x_in, y_in);
            node = trigrid_field_[node_idx.row][node_idx.col][node_idx.tri];
            return node;
        };

        TriGridNode<PointType> getTriGridNode(const TriGridIdx& tgf_idx){
            TriGridNode<PointType> node;
            node = trigrid_field_[tgf_idx.row][tgf_idx.col][tgf_idx.tri];
            return node;
        };

        bool is_traversable(const float& x_in, const float& y_in){
            TriGridNode<PointType> node = getTriGridNode(x_in, y_in);
            if (node.node_type == GROUND){
                return true;
            } else{
                return false;
            }
        };

        pcl::PointCloud<PointType> getObstaclePC(){
            pcl::PointCloud<PointType> cloud_obstacle;
            cloud_obstacle = ptCloud_tgfwise_obstacle_;
            return cloud_obstacle;
        };

    private:
        double tgf_max_x, tgf_max_y, tgf_min_x, tgf_min_y;
        double rows_, cols_;

        void initTriGridField(TriGridField<PointType>& tgf_in){
            // print("Initializing TriGridField...");

            tgf_max_x = MAX_RANGE_;
            tgf_max_y = MAX_RANGE_;

            tgf_min_x = -MAX_RANGE_;
            tgf_min_y = -MAX_RANGE_;

            rows_ = (int)(tgf_max_x - tgf_min_x) / TGF_RESOLUTION_;
            cols_ = (int)(tgf_max_y - tgf_min_y) / TGF_RESOLUTION_;
            empty_cloud_.clear();
            empty_cloud_.reserve(PTCLOUD_SIZE);
            
            // Set Node structure
            empty_trigrid_node_.node_type = UNKNOWN;
            empty_trigrid_node_.ptCloud.clear();
            empty_trigrid_node_.ptCloud.reserve(NODEWISE_PTCLOUDSIZE);

            empty_trigrid_node_.is_curr_data = false;
            empty_trigrid_node_.need_recheck = false;
            empty_trigrid_node_.is_visited = false;
            empty_trigrid_node_.is_rejection = false;

            empty_trigrid_node_.check_life = 10;
            empty_trigrid_node_.depth = -1;

            empty_trigrid_node_.normal;
            empty_trigrid_node_.mean_pt;
            empty_trigrid_node_.d = 0;
            
            empty_trigrid_node_.singular_values;
            empty_trigrid_node_.eigen_vectors;
            empty_trigrid_node_.weight = 0;

            empty_trigrid_node_.th_dist_d = 0;
            empty_trigrid_node_.th_outlier_d = 0;

            // Set TriGridField
            tgf_in.clear();
            std::vector<GridNode<PointType>> vec_gridnode;

            for (int i = 0; i < 4 ; i ++) 
                empty_grid_nodes_.emplace_back(empty_trigrid_node_);
                
            for (int i=0; i< cols_; i++){ vec_gridnode.emplace_back(empty_grid_nodes_);}
            for (int j=0; j< rows_; j++){ tgf_in.emplace_back(vec_gridnode);}

            return;
        };

        void initTriGridCorners(std::vector<std::vector<TriGridCorner>>& trigrid_corners_in,
                                std::vector<std::vector<TriGridCorner>>& trigrid_centers_in){
            // print("Initializing TriGridCorners...");

            // Set TriGridCorner
            empty_trigrid_corner_.x = empty_trigrid_corner_.y = 0.0;
            empty_trigrid_corner_.zs.clear();
            empty_trigrid_corner_.zs.reserve(8);
            empty_trigrid_corner_.weights.clear();
            empty_trigrid_corner_.weights.reserve(8);

            empty_trigrid_center_.x = empty_trigrid_center_.y = 0.0;
            empty_trigrid_center_.zs.clear();
            empty_trigrid_center_.zs.reserve(4);
            empty_trigrid_center_.weights.clear();
            empty_trigrid_center_.weights.reserve(4);

            trigrid_corners_in.clear();
            trigrid_centers_in.clear();

            // Set Corners and Centers
            std::vector<TriGridCorner> col_corners;
            std::vector<TriGridCorner> col_centers;
            for (int i=0; i< cols_; i++){
                col_corners.emplace_back(empty_trigrid_corner_);
                col_centers.emplace_back(empty_trigrid_center_);
            }
            col_corners.emplace_back(empty_trigrid_corner_);

            for (int j=0; j< rows_; j++){
                trigrid_corners_in.emplace_back(col_corners);
                trigrid_centers_in.emplace_back(col_centers);
            }
            trigrid_corners_in.emplace_back(col_corners);

            return;
        };

        void clearTriGridField(TriGridField<PointType> &tgf_in){
            // print("Clearing TriGridField...");

            for (int r_i = 0; r_i < rows_; r_i++){
            for (int c_i = 0; c_i < cols_; c_i++){
                tgf_in[r_i][c_i] = empty_grid_nodes_;
            }
            }
            return;
        };

        void clearTriGridCorners(std::vector<std::vector<TriGridCorner>>& trigrid_corners_in,
                                std::vector<std::vector<TriGridCorner>>& trigrid_centers_in){
            // print("Clearing TriGridCorners...");

            TriGridCorner tmp_corner = empty_trigrid_corner_;
            TriGridCorner tmp_center = empty_trigrid_center_;
            for (int r_i = 0; r_i < rows_+1; r_i++){
            for (int c_i = 0; c_i < cols_+1; c_i++){
                tmp_corner.x = (r_i)*TGF_RESOLUTION_+tgf_min_x; tmp_corner.y = (c_i)*TGF_RESOLUTION_+tgf_min_y;
                tmp_corner.zs.clear();
                tmp_corner.weights.clear();
                trigrid_corners_in[r_i][c_i] = tmp_corner;
                if (r_i < rows_ && c_i < cols_) {
                    tmp_center.x = (r_i+0.5)*TGF_RESOLUTION_+tgf_min_x; tmp_center.y = (c_i+0.5)*TGF_RESOLUTION_+tgf_min_y;
                    tmp_center.zs.clear();
                    tmp_center.weights.clear();
                    trigrid_centers_in[r_i][c_i] = tmp_center;
                }
            }
            }
            return;
        };
        
        double xy_2Dradius(double x, double y){
            return sqrt(x*x + y*y);
        };

        bool filterPoint(const PointType &pt_in){
            double xy_range = xy_2Dradius(pt_in.x, pt_in.y);
            if (xy_range >= MAX_RANGE_ || xy_range <= MIN_RANGE_) return true;

            return false;
        }

        void embedCloudToTriGridField(const pcl::PointCloud<PointType>& cloud_in, TriGridField<PointType>& tgf_out) {
            // print("Embedding PointCloud to TriGridField...");

            for (auto const &pt: cloud_in.points){
                if (filterPoint(pt)){
                    ptCloud_tgfwise_outliers_.points.push_back(pt);
                    continue;   
                }

                int r_i = (pt.x - tgf_min_x)/TGF_RESOLUTION_;
                int c_i = (pt.y - tgf_min_y)/TGF_RESOLUTION_;

                if (r_i < 0 || r_i >= rows_ || c_i < 0 || c_i >= cols_) {
                    ptCloud_tgfwise_outliers_.points.push_back(pt);
                    continue;
                }

                double angle = atan2(pt.y-(c_i*TGF_RESOLUTION_ + TGF_RESOLUTION_/2 + tgf_min_y), pt.x-(r_i*TGF_RESOLUTION_ + TGF_RESOLUTION_/2 + tgf_min_x));
                if (angle>=(M_PI/4) && angle <(3*M_PI/4)){
                    // left side
                    tgf_out[r_i][c_i][1].ptCloud.push_back(pt);
                    if(!tgf_out[r_i][c_i][1].is_curr_data) {tgf_out[r_i][c_i][1].is_curr_data = true;}
                } else if (angle>=(-M_PI/4) && angle <(M_PI/4)){
                    // upper side
                    tgf_out[r_i][c_i][0].ptCloud.push_back(pt);
                    if (!tgf_out[r_i][c_i][0].is_curr_data){tgf_out[r_i][c_i][0].is_curr_data = true;}
                    
                } else if (angle>=(-3*M_PI/4) && angle <(-M_PI/4)){
                    // right side
                    tgf_out[r_i][c_i][3].ptCloud.push_back(pt);
                    if (!tgf_out[r_i][c_i][3].is_curr_data) {tgf_out[r_i][c_i][3].is_curr_data = true;}
                } else{
                    // lower side
                    tgf_out[r_i][c_i][2].ptCloud.push_back(pt);
                    if (!tgf_out[r_i][c_i][2].is_curr_data) {tgf_out[r_i][c_i][2].is_curr_data = true;}
                }
            }

            return;
        };

        void extractInitialSeeds(const pcl::PointCloud<PointType>& p_sorted, pcl::PointCloud<PointType>& init_seeds){
            //function for uniform mode
            init_seeds.points.clear();

            // LPR is the mean of Low Point Representative
            double sum = 0;
            int cnt = 0;

            // Calculate the mean height value.
            for (int i=0; i< (int) p_sorted.points.size() && cnt<NUM_LRP_; i++){
                sum += p_sorted.points[i].z;
                cnt++;
            }

            double lpr_height = cnt!=0?sum/cnt:0;

            for(int i=0 ; i< (int) p_sorted.points.size() ; i++){
                if(p_sorted.points[i].z < lpr_height + TH_SEEDS_){
                    if (p_sorted.points[i].z < lpr_height-TH_OUTLIER_) continue;
                    init_seeds.points.push_back(p_sorted.points[i]);
                }
            }

            return;
        }

        void estimatePlanarModel(const pcl::PointCloud<PointType>& ground_in, TriGridNode<PointType>& node_out) {

            // function for uniform mode
            Eigen::Matrix3f cov_;
            Eigen::Vector4f pc_mean_;
            pcl::computeMeanAndCovarianceMatrix(ground_in, cov_, pc_mean_);

            // Singular Value Decomposition: SVD
            Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov_, Eigen::DecompositionOptions::ComputeFullU);

            // Use the least singular vector as normal
            node_out.eigen_vectors = svd.matrixU();
            if (node_out.eigen_vectors.col(2)(2,0)<0) {
                node_out.eigen_vectors.col(0)*= -1;
                node_out.eigen_vectors.col(2)*= -1;
            }
            node_out.normal = node_out.eigen_vectors.col(2);
            node_out.singular_values = svd.singularValues();

            // mean ground seeds value
            node_out.mean_pt = pc_mean_.head<3>();

            // according to noraml.T*[x,y,z] = -d
            node_out.d = -(node_out.normal.transpose()*node_out.mean_pt)(0,0);

            // set distance theshold to 'th_dist - d'
            node_out.th_dist_d = TH_DIST_ - node_out.d;
            node_out.th_outlier_d = -node_out.d - TH_OUTLIER_;

            return;
        }    

        void modelPCAbasedTerrain(TriGridNode<PointType>& node_in) {
            // Initailization
            if (!ptCloud_nodewise_ground_.empty()) ptCloud_nodewise_ground_.clear();
            
            // Tri Grid Initialization
            // When to initialize the planar model, we don't have prior. so outlier is removed in heuristic parameter.
            pcl::PointCloud<PointType> sort_ptCloud = node_in.ptCloud;

            // sort in z-coordinate
            sort(sort_ptCloud.points.begin(), sort_ptCloud.end(), point_z_cmp<PointType>);

            // Set init seeds
            extractInitialSeeds(sort_ptCloud, ptCloud_nodewise_ground_);
            
            Eigen::MatrixXf points(sort_ptCloud.points.size(),3);
            int j = 0;
            for (auto& p:sort_ptCloud.points){
                points.row(j++)<<p.x, p.y, p.z;
            }
            // Extract Ground
            for (int i =0; i < NUM_ITER_; i++){
                estimatePlanarModel(ptCloud_nodewise_ground_, node_in);
                if(ptCloud_nodewise_ground_.size() < 3){
    
                    node_in.node_type = NONGROUND;
                    break;
                }
                ptCloud_nodewise_ground_.clear();
                // threshold filter
                Eigen::VectorXf result = points*node_in.normal;
                for (int r = 0; r<result.rows(); r++){
                    if (i < NUM_ITER_-1){
                        if (result[r]<node_in.th_dist_d){
                            ptCloud_nodewise_ground_.push_back(sort_ptCloud.points[r]);
                        }
                    } else {
                        // Final interation
                        if (node_in.normal(2,0) < TH_NORMAL_ ){
                            node_in.node_type = NONGROUND;
                        } else {
                            node_in.node_type = GROUND;
                        }
                    }
                }
            }

            return;
        }

        double calcNodeWeight(const TriGridNode<PointType>& node_in){
            double weight = 0;
            
            // weight = (node_in.singular_values[0]/node_in.singular_values[2] + node_in.singular_values[1]/node_in.singular_values[2])/(node_in.singular_values[0]/node_in.singular_values[1]);
            weight = (node_in.singular_values[0] + node_in.singular_values[1])*node_in.singular_values[1]/(node_in.singular_values[0]*node_in.singular_values[2]+0.001);

            return weight;
        }

        void modelNodeWiseTerrain(TriGridField<PointType>& tgf_in) {
            // print("Node-wise Terrain Modeling...");

            for (int r_i = 0; r_i < rows_; r_i++){
            for (int c_i = 0; c_i < cols_; c_i++){
            for (int s_i = 0; s_i < 4; s_i++){
                if (tgf_in[r_i][c_i][s_i].is_curr_data){
                    if (tgf_in[r_i][c_i][s_i].ptCloud.size() < NUM_MIN_POINTS_){
                        tgf_in[r_i][c_i][s_i].node_type = UNKNOWN;
                        continue;                    
                    } else {
                        modelPCAbasedTerrain(tgf_in[r_i][c_i][s_i]);
                        if (tgf_in[r_i][c_i][s_i].node_type == GROUND){ tgf_in[r_i][c_i][s_i].weight = calcNodeWeight(tgf_in[r_i][c_i][s_i]); }
                    }
                }
            }
            }
            }

            return;
        };

        void findDominantNode(const TriGridField<PointType>& tgf_in, TriGridIdx& node_idx_out) {
            // Find the dominant node
            // std::cout << "Find the dominant node..." << std::endl;
            TriGridIdx max_tri_idx;
            TriGridIdx ego_idx;
            ego_idx.row = (int)((0-tgf_min_x)/TGF_RESOLUTION_);
            ego_idx.col = (int)((0-tgf_min_y)/TGF_RESOLUTION_);
            ego_idx.tri = 0;
            
            max_tri_idx = ego_idx;
            for (int r_i = ego_idx.row - 2; r_i < ego_idx.row + 2; r_i++){
            for (int c_i = ego_idx.col - 2; c_i < ego_idx.col + 2; c_i++){
            for (int s_i = 0; s_i < 4; s_i++){
                if (tgf_in[r_i][c_i][s_i].is_curr_data){
                    if (tgf_in[r_i][c_i][s_i].node_type == GROUND){
                        if (tgf_in[r_i][c_i][s_i].weight > tgf_in[max_tri_idx.row][max_tri_idx.row][max_tri_idx.tri].weight){
                            max_tri_idx.row = r_i;
                            max_tri_idx.col = c_i;
                            max_tri_idx.tri = s_i;
                        }
                    }
                }
            }
            }    
            }
            node_idx_out = max_tri_idx;
            return;
        };

        void searchNeighborNodes(const TriGridIdx &cur_idx, std::vector<TriGridIdx> &neighbor_idxs) {
            neighbor_idxs.clear();
            neighbor_idxs.reserve(14);
            int r_i = cur_idx.row;
            int c_i = cur_idx.col;
            int s_i = cur_idx.tri;

            std::vector<TriGridIdx> tmp_neighbors;
            tmp_neighbors.clear();
            tmp_neighbors.reserve(14);
            
            TriGridIdx neighbor_idx;
            for (int s_i = 0; s_i < 4 ; s_i++){
                if (s_i == cur_idx.tri) continue;

                neighbor_idx = cur_idx;
                neighbor_idx.tri = s_i;
                tmp_neighbors.push_back(neighbor_idx);
            }

            switch (s_i) {
                case 0:
                    tmp_neighbors.push_back({r_i+1, c_i+1, 2});
                    tmp_neighbors.push_back({r_i+1, c_i+1, 3});
                    tmp_neighbors.push_back({r_i+1, c_i  , 1});
                    tmp_neighbors.push_back({r_i+1, c_i  , 2});
                    tmp_neighbors.push_back({r_i+1, c_i  , 3});
                    tmp_neighbors.push_back({r_i+1, c_i-1, 1});
                    tmp_neighbors.push_back({r_i+1, c_i-1, 2});
                    tmp_neighbors.push_back({r_i  , c_i+1, 0});
                    tmp_neighbors.push_back({r_i  , c_i+1, 3});
                    tmp_neighbors.push_back({r_i  , c_i-1, 0});
                    tmp_neighbors.push_back({r_i  , c_i-1, 1});
                    break;
                case 1:
                    tmp_neighbors.push_back({r_i+1, c_i+1, 2});
                    tmp_neighbors.push_back({r_i+1, c_i+1, 3});
                    tmp_neighbors.push_back({r_i+1, c_i  , 1});
                    tmp_neighbors.push_back({r_i+1, c_i  , 2});
                    tmp_neighbors.push_back({r_i  , c_i+1, 0});
                    tmp_neighbors.push_back({r_i  , c_i+1, 2});
                    tmp_neighbors.push_back({r_i  , c_i+1, 3});
                    tmp_neighbors.push_back({r_i-1, c_i+1, 0});
                    tmp_neighbors.push_back({r_i-1, c_i+1, 3});
                    tmp_neighbors.push_back({r_i-1, c_i+1, 0});
                    tmp_neighbors.push_back({r_i-1, c_i  , 1});
                    break;
                case 2:
                    tmp_neighbors.push_back({r_i  , c_i+1, 2});
                    tmp_neighbors.push_back({r_i  , c_i+1, 3});
                    tmp_neighbors.push_back({r_i  , c_i-1, 1});
                    tmp_neighbors.push_back({r_i  , c_i-1, 2});
                    tmp_neighbors.push_back({r_i-1, c_i+1, 0});
                    tmp_neighbors.push_back({r_i-1, c_i+1, 3});
                    tmp_neighbors.push_back({r_i-1, c_i  , 0});
                    tmp_neighbors.push_back({r_i-1, c_i  , 1});
                    tmp_neighbors.push_back({r_i-1, c_i  , 3});
                    tmp_neighbors.push_back({r_i-1, c_i-1, 0});
                    tmp_neighbors.push_back({r_i-1, c_i-1, 1});
                    break;
                case 3:
                    tmp_neighbors.push_back({r_i+1, c_i  , 2});
                    tmp_neighbors.push_back({r_i+1, c_i  , 3});
                    tmp_neighbors.push_back({r_i+1, c_i-1, 1});
                    tmp_neighbors.push_back({r_i+1, c_i-1, 2});
                    tmp_neighbors.push_back({r_i  , c_i-1, 0});
                    tmp_neighbors.push_back({r_i  , c_i-1, 1});
                    tmp_neighbors.push_back({r_i  , c_i-1, 2});
                    tmp_neighbors.push_back({r_i-1, c_i  , 0});
                    tmp_neighbors.push_back({r_i-1, c_i  , 3});
                    tmp_neighbors.push_back({r_i-1, c_i-1, 0});
                    tmp_neighbors.push_back({r_i-1, c_i-1, 1});
                    break;
                default:
                    break;
            }

            for (int n_i = 0 ; n_i < (int) tmp_neighbors.size() ; n_i++) {

                if (tmp_neighbors[n_i].row >= rows_ || tmp_neighbors[n_i].row < 0) {
                    continue;
                }

                if (tmp_neighbors[n_i].col >= cols_ || tmp_neighbors[n_i].col < 0) {
                    continue;
                }

                neighbor_idxs.push_back(tmp_neighbors[n_i]);
            }
        }

        void searchAdjacentNodes(const TriGridIdx &cur_idx, std::vector<TriGridIdx> &adjacent_idxs) {
            adjacent_idxs.clear();
            adjacent_idxs.reserve(3);
            int r_i = cur_idx.row;
            int c_i = cur_idx.col;
            int s_i = cur_idx.tri;

            std::vector<TriGridIdx> tmp_neighbors;
            tmp_neighbors.clear();
            tmp_neighbors.reserve(3);
            
            TriGridIdx neighbor_idx;

            switch (s_i) {
                case 0:
                    tmp_neighbors.push_back({r_i+1, c_i, 2});
                    tmp_neighbors.push_back({r_i  , c_i, 3});
                    tmp_neighbors.push_back({r_i  , c_i, 1});
     
                    break;
                case 1:
                    tmp_neighbors.push_back({r_i, c_i+1, 3});
                    tmp_neighbors.push_back({r_i, c_i  , 0});
                    tmp_neighbors.push_back({r_i, c_i  , 2});                    
                    break;
                case 2:
                    tmp_neighbors.push_back({r_i-1, c_i, 0});
                    tmp_neighbors.push_back({r_i  , c_i, 1});
                    tmp_neighbors.push_back({r_i  , c_i, 3});
                    break;
                case 3:
                    tmp_neighbors.push_back({r_i, c_i-1, 1});
                    tmp_neighbors.push_back({r_i, c_i  , 2});
                    tmp_neighbors.push_back({r_i, c_i  , 0});
                    break;
                default:
                    break;
            }

            for (int n_i = 0 ; n_i < (int) tmp_neighbors.size() ; n_i++) {

                if (tmp_neighbors[n_i].row >= rows_ || tmp_neighbors[n_i].row < 0) {
                    continue;
                }

                if (tmp_neighbors[n_i].col >= cols_ || tmp_neighbors[n_i].col < 0) {
                    continue;
                }

                adjacent_idxs.push_back(tmp_neighbors[n_i]);
            }
        }

        bool LocalConvecityConcavity(const TriGridField<PointType> &tgf, const TriGridIdx &cur_node_idx, const TriGridIdx &neighbor_idx, 
                                    double & thr_local_normal, double & thr_local_dist) {
            TriGridNode<PointType> current_node = tgf[cur_node_idx.row][cur_node_idx.col][cur_node_idx.tri];
            TriGridNode<PointType> neighbor_node = tgf[neighbor_idx.row][neighbor_idx.col][neighbor_idx.tri];

            Eigen::Vector3f normal_src = current_node.normal; 
            Eigen::Vector3f normal_tgt = neighbor_node.normal; 
            Eigen::Vector3f meanPt_diff_s2t = neighbor_node.mean_pt - current_node.mean_pt;

            double diff_norm = meanPt_diff_s2t.norm();
            double dist_s2t = normal_src.dot(meanPt_diff_s2t);
            double dist_t2s = normal_tgt.dot(-meanPt_diff_s2t);

            double normal_similarity = normal_src.dot(normal_tgt);
            double TH_NORMAL_cos_similarity = sin(diff_norm*thr_local_normal);
            if ((normal_similarity < (1-TH_NORMAL_cos_similarity))) {
                return false;
            }

            double TH_DIST_to_planar = diff_norm*sin(thr_local_dist);
            if ( (abs(dist_s2t) > TH_DIST_to_planar || abs(dist_t2s) > TH_DIST_to_planar) ) {
                return false;
            }

            return true;
        }

        void BreadthFirstTraversableGraphSearch(TriGridField<PointType>& tgf_in) {

            // Find the dominant node
            std::queue<TriGridIdx> searching_idx_queue;
            TriGridIdx dominant_node_idx;
            findDominantNode(tgf_in, dominant_node_idx);
            tgf_in[dominant_node_idx.row][dominant_node_idx.col][dominant_node_idx.tri].is_visited = true;
            tgf_in[dominant_node_idx.row][dominant_node_idx.col][dominant_node_idx.tri].depth = 0;
            tgf_in[dominant_node_idx.row][dominant_node_idx.col][dominant_node_idx.tri].node_type = GROUND;

            searching_idx_queue.push(dominant_node_idx);

            double max_planar_height = 0;
            trigrid_edges_.clear();
            trigrid_edges_.reserve(rows_*cols_*4);
            TriGridEdge cur_edge;
            TriGridIdx current_node_idx;
            while (!searching_idx_queue.empty()){
                // set current node
                current_node_idx = searching_idx_queue.front();
                searching_idx_queue.pop();

                // search the neighbor nodes
                std::vector<TriGridIdx> neighbor_idxs;
                searchNeighborNodes(current_node_idx, neighbor_idxs);
                
                // set the traversable edges
                for (int i = 0; i < (int) neighbor_idxs.size(); i++){
                    // if the neighbor node is traversable, add it to the queue

                    TriGridIdx n_i = neighbor_idxs[i];


                    if (tgf_in[n_i.row][n_i.col][n_i.tri].depth >=0){
                        continue;
                    }

                    if (tgf_in[n_i.row][n_i.col][n_i.tri].is_visited) {
                        if (!tgf_in[n_i.row][n_i.col][n_i.tri].need_recheck){
                            continue;
                        } else {
                            if (tgf_in[n_i.row][n_i.col][n_i.tri].check_life <= 0){
                                continue;
                            }
                        }
                        continue;
                    } else {
                        if (tgf_in[n_i.row][n_i.col][n_i.tri].node_type != GROUND) {
                        continue;
                        }
                    }

                    tgf_in[n_i.row][n_i.col][n_i.tri].is_visited =true;
                    
                    if (!LocalConvecityConcavity(tgf_in, current_node_idx, n_i, TH_LCC_NORMAL_SIMILARITY_, TH_LCC_PLANAR_MODEL_DIST_)) {
                        tgf_in[n_i.row][n_i.col][n_i.tri].is_rejection = true;
                        tgf_in[n_i.row][n_i.col][n_i.tri].node_type = NONGROUND;

                        if(tgf_in[n_i.row][n_i.col][n_i.tri].check_life > 0) {
                            tgf_in[n_i.row][n_i.col][n_i.tri].check_life -=1;
                            tgf_in[n_i.row][n_i.col][n_i.tri].need_recheck = true;
                        } else {
                            tgf_in[n_i.row][n_i.col][n_i.tri].need_recheck = false;
                        }
                        continue;
                    }

                    if (max_planar_height < tgf_in[n_i.row][n_i.col][n_i.tri].mean_pt[2]) max_planar_height = tgf_in[n_i.row][n_i.col][n_i.tri].mean_pt[2];

                    tgf_in[n_i.row][n_i.col][n_i.tri].node_type = GROUND;
                    tgf_in[n_i.row][n_i.col][n_i.tri].is_rejection = false;
                    tgf_in[n_i.row][n_i.col][n_i.tri].depth = tgf_in[current_node_idx.row][current_node_idx.col][current_node_idx.tri].depth + 1;

                    if (VIZ_MDOE_){
                        cur_edge.Pair.first = current_node_idx;
                        cur_edge.Pair.second = n_i;
                        cur_edge.is_traversable = true;
                        trigrid_edges_.push_back(cur_edge);
                    }

                    searching_idx_queue.push(n_i);
                }

                if (searching_idx_queue.empty()){
                    // set the new dominant node
                    for (int r_i = 0; r_i < rows_; r_i++) {
                    for (int c_i = 0; c_i < cols_; c_i++) {
                    for (int s_i = 0; s_i < (int) tgf_in[r_i][c_i].size() ; s_i++){
                        if (tgf_in[r_i][c_i][s_i].is_visited) { continue; }

                        if (tgf_in[r_i][c_i][s_i].node_type != GROUND ) { continue; }

                        // if (tgf_in[r_i][c_i][s_i].mean_pt[2] >= max_planar_height+1) { continue; }

                        if (tgf_in[r_i][c_i][s_i].depth >= 0) { continue; }

                        // if (tgf_in[r_i][c_i][s_i].weight > 5*TH_WEIGHT_){
                            tgf_in[r_i][c_i][s_i].depth = 0;
                            tgf_in[r_i][c_i][s_i].is_visited = true;

                            TriGridIdx new_dominant_idx = {r_i, c_i, s_i};
                            searching_idx_queue.push(new_dominant_idx);
                        // }
                    }
                    }
                    }
                }
            }
            return;
        };


        double getCornerWeight(const TriGridNode<PointType>& node_in, const pcl::PointXYZ &tgt_corner){
            double xy_dist = sqrt( (node_in.mean_pt[0]-tgt_corner.x)*(node_in.mean_pt[0]-tgt_corner.x)+(node_in.mean_pt[1]-tgt_corner.y)*(node_in.mean_pt[1]-tgt_corner.y) );
            return (node_in.weight/xy_dist);
        }

        void setTGFCornersCenters(const TriGridField<PointType>& tgf_in,
                                std::vector<std::vector<TriGridCorner>>& trigrid_corners_out,
                                std::vector<std::vector<TriGridCorner>>& trigrid_centers_out) {
            pcl::PointXYZ corner_TL, corner_BL, corner_BR, corner_TR, corner_C;

            for (int r_i = 0; r_i<rows_; r_i++){
            for (int c_i = 0; c_i<cols_; c_i++){
                corner_TL.x = trigrid_corners_out[r_i+1][c_i+1].x; corner_TL.y = trigrid_corners_out[r_i+1][c_i+1].y;   // LT
                corner_BL.x = trigrid_corners_out[r_i][c_i+1].x;   corner_BL.y = trigrid_corners_out[r_i][c_i+1].y;     // LL
                corner_BR.x = trigrid_corners_out[r_i][c_i].x;     corner_BR.y = trigrid_corners_out[r_i][c_i].y;       // RL
                corner_TR.x = trigrid_corners_out[r_i+1][c_i].x;   corner_TR.y = trigrid_corners_out[r_i+1][c_i].y;     // RT
                corner_C.x = trigrid_centers_out[r_i][c_i].x;    corner_C.y = trigrid_centers_out[r_i][c_i].y;       // Center

                for (int s_i = 0; s_i< (int) tgf_in[r_i][c_i].size();s_i++){
                    if (tgf_in[r_i][c_i][s_i].node_type != GROUND) { continue; }
                    if (tgf_in[r_i][c_i][s_i].is_rejection) { continue; }
                    if (tgf_in[r_i][c_i][s_i].depth == -1) { continue; }

                    switch(s_i){
                        case 0: // upper Tri-grid bin
                            // RT / LT / C
                            trigrid_corners_out[r_i+1][c_i].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_TR.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_TR.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_corners_out[r_i+1][c_i].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_TR));

                            trigrid_corners_out[r_i+1][c_i+1].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_TL.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_TL.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_corners_out[r_i+1][c_i+1].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_TL));

                            trigrid_centers_out[r_i][c_i].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_C.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_C.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_centers_out[r_i][c_i].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_C));

                            break;
                        case 1: // left Tri-grid bin
                            // LT / LL / C
                            trigrid_corners_out[r_i+1][c_i+1].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_TL.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_TL.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_corners_out[r_i+1][c_i+1].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_TL));

                            trigrid_corners_out[r_i][c_i+1].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_BL.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_BL.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_corners_out[r_i][c_i+1].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_BL));

                            trigrid_centers_out[r_i][c_i].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_C.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_C.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_centers_out[r_i][c_i].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_C));

                            break;
                        case 2: // lower Tri-grid bin
                            // LL / RL / C
                            trigrid_corners_out[r_i][c_i+1].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_BL.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_BL.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_corners_out[r_i][c_i+1].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_BL));

                            trigrid_corners_out[r_i][c_i].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_BR.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_BR.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_corners_out[r_i][c_i].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_BR));

                            trigrid_centers_out[r_i][c_i].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_C.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_C.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_centers_out[r_i][c_i].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_C));
                            
                            break;
                        case 3: // right Tri-grid bin
                            // RL / RT / C
                            trigrid_corners_out[r_i][c_i].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_BR.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_BR.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_corners_out[r_i][c_i].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_BR));

                            trigrid_corners_out[r_i+1][c_i].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_TR.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_TR.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_corners_out[r_i+1][c_i].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_TR));

                            trigrid_centers_out[r_i][c_i].zs.push_back( (-tgf_in[r_i][c_i][s_i].normal(0,0)*corner_C.x - tgf_in[r_i][c_i][s_i].normal(1,0)*corner_C.y-tgf_in[r_i][c_i][s_i].d)/tgf_in[r_i][c_i][s_i].normal(2,0) );
                            trigrid_centers_out[r_i][c_i].weights.push_back(getCornerWeight(tgf_in[r_i][c_i][s_i],corner_C));
                            
                            break;
                        default:
                            break;
                    }
                }
            }
            }
            return;
        };
        
        TriGridCorner getMeanCorner(const TriGridCorner &corners_in){
            // get the mean of the corners

            TriGridCorner corners_out;
            corners_out.x = corners_in.x;
            corners_out.y = corners_in.y;
            corners_out.zs.clear();
            corners_out.weights.clear();

            double weighted_sum_z = 0.0;
            double sum_w = 0.0;
            for (int i = 0; i < (int) corners_in.zs.size(); i++){
                weighted_sum_z += corners_in.zs[i]*corners_in.weights[i];
                sum_w += corners_in.weights[i];
            }

            corners_out.zs.push_back(weighted_sum_z/sum_w);
            corners_out.weights.push_back(sum_w);

            return corners_out;
        }

        void updateTGFCornersCenters(std::vector<std::vector<TriGridCorner>>& trigrid_corners_out,
                                    std::vector<std::vector<TriGridCorner>>& trigrid_centers_out) {

            // update corners
            TriGridCorner updated_corner = empty_trigrid_corner_;
            for (int r_i = 0; r_i < rows_ +1; r_i++) {
            for (int c_i = 0; c_i < cols_ +1; c_i++) {
                if (trigrid_corners_out[r_i][c_i].zs.size() > 0 && trigrid_corners_out[r_i][c_i].weights.size() > 0) {
                    updated_corner = getMeanCorner(trigrid_corners_out[r_i][c_i]);
                    trigrid_corners_out[r_i][c_i] = updated_corner;
                } else {
                    trigrid_corners_out[r_i][c_i].zs.clear();
                    trigrid_corners_out[r_i][c_i].weights.clear();
                }
            }
            }        

            // update centers
            TriGridCorner updated_center = empty_trigrid_center_;
            for (int r_i = 0; r_i < rows_; r_i++) {
            for (int c_i = 0; c_i < cols_; c_i++) {
                if (trigrid_centers_out[r_i][c_i].zs.size() > 0 && trigrid_centers_out[r_i][c_i].weights.size() > 0) {
                    updated_center = getMeanCorner(trigrid_centers_out[r_i][c_i]);
                    trigrid_centers_out[r_i][c_i] = updated_center;
                    // trigrid_centers_out[r_i][c_i].z = get_mean(trigrid_centers_out[r_i][c_i].zs,trigrid_centers_out[r_i][c_i].weights);
                } else {
                    trigrid_centers_out[r_i][c_i].zs.clear();
                    trigrid_centers_out[r_i][c_i].weights.clear();
                }
            }
            }

            return;
        };

        Eigen::Vector3f convertCornerToEigen(TriGridCorner &corner_in) {
            Eigen::Vector3f corner_out;
            if (corner_in.zs.size() != corner_in.weights.size()){
                std::cout << "ERROR in corners" << std::endl;
            }
            corner_out[0] = corner_in.x;
            corner_out[1] = corner_in.y;
            corner_out[2] = corner_in.zs[0];
            return corner_out;
        };

        void revertTraversableNodes(std::vector<std::vector<TriGridCorner>>& trigrid_corners_in,
                                    std::vector<std::vector<TriGridCorner>>& trigrid_centers_in, 
                                    TriGridField<PointType>& tgf_out) {
            Eigen::Vector3f refined_corner_1, refined_corner_2, refined_center;
            for (int r_i = 0; r_i < rows_; r_i++) {
            for (int c_i = 0; c_i < cols_; c_i++) {
            for (int s_i = 0; s_i < (int) tgf_out[r_i][c_i].size(); s_i++) {
                // set the corners for the current trigrid node
                switch (s_i)
                {
                case 0:
                    if ( trigrid_corners_in[r_i+1][c_i].zs.size()==0 || trigrid_corners_in[r_i+1][c_i+1].zs.size()==0  || trigrid_centers_in[r_i][c_i].zs.size()==0 ){
                        if (tgf_out[r_i][c_i][s_i].node_type != NONGROUND){
                            tgf_out[r_i][c_i][s_i].node_type = UNKNOWN;
                        }
                        continue;
                    }
                    refined_corner_1 = convertCornerToEigen(trigrid_corners_in[r_i+1][c_i]);
                    refined_corner_2 = convertCornerToEigen(trigrid_corners_in[r_i+1][c_i+1]);
                    refined_center = convertCornerToEigen(trigrid_centers_in[r_i][c_i]);
                    break;
                case 1:
                    if ( trigrid_corners_in[r_i+1][c_i+1].zs.size()==0 || trigrid_corners_in[r_i][c_i+1].zs.size()==0  || trigrid_centers_in[r_i][c_i].zs.size()==0 ){
                        if (tgf_out[r_i][c_i][s_i].node_type != NONGROUND){
                            tgf_out[r_i][c_i][s_i].node_type = UNKNOWN;
                        }
                        continue;
                    }
                    refined_corner_1 = convertCornerToEigen(trigrid_corners_in[r_i+1][c_i+1]);
                    refined_corner_2 = convertCornerToEigen(trigrid_corners_in[r_i][c_i+1]);
                    refined_center = convertCornerToEigen(trigrid_centers_in[r_i][c_i]);    
                    break;
                case 2:
                    if ( trigrid_corners_in[r_i][c_i+1].zs.size()==0 || trigrid_corners_in[r_i][c_i].zs.size()==0  || trigrid_centers_in[r_i][c_i].zs.size()==0 ){
                        if (tgf_out[r_i][c_i][s_i].node_type != NONGROUND){
                            tgf_out[r_i][c_i][s_i].node_type = UNKNOWN;
                        }
                        continue;
                    }
                    refined_corner_1 = convertCornerToEigen(trigrid_corners_in[r_i][c_i+1]);
                    refined_corner_2 = convertCornerToEigen(trigrid_corners_in[r_i][c_i]);
                    refined_center = convertCornerToEigen(trigrid_centers_in[r_i][c_i]);
                    break;
                case 3:
                    if ( trigrid_corners_in[r_i][c_i].zs.size()==0 || trigrid_corners_in[r_i+1][c_i].zs.size()==0  || trigrid_centers_in[r_i][c_i].zs.size()==0 ){
                        if (tgf_out[r_i][c_i][s_i].node_type != NONGROUND){
                            tgf_out[r_i][c_i][s_i].node_type = UNKNOWN;
                        }
                        continue;
                    }
                    refined_corner_1 = convertCornerToEigen(trigrid_corners_in[r_i][c_i]);
                    refined_corner_2 = convertCornerToEigen(trigrid_corners_in[r_i+1][c_i]);
                    refined_center = convertCornerToEigen(trigrid_centers_in[r_i][c_i]);
                    break;
                default:
                    std::cout << "WRONG tri-grid indexing" << std::endl;
                    break;
                }

                // calculate the refined planar model in the node
                Eigen::Vector3f udpated_normal = (refined_corner_1-refined_center).cross(refined_corner_2-refined_center);
                udpated_normal /= udpated_normal.norm();
                // if (udpated_normal[2] < 0){
                //     std::cout << "Origin normal: " << tgf_out[r_i][c_i][s_i].normal << std::endl;
                //     std::cout << "Update normal: " << udpated_normal << std::endl;
                // }
                if (udpated_normal(2,0) < TH_NORMAL_ ){   // non-planar
                    tgf_out[r_i][c_i][s_i].normal = udpated_normal;
                    tgf_out[r_i][c_i][s_i].node_type = NONGROUND;
                } else {    
                    // planar
                    Eigen::Vector3f updated_mean_pt;
                    updated_mean_pt[0] = (refined_corner_1[0] + refined_corner_2[0] + refined_center[0])/3;
                    updated_mean_pt[1] = (refined_corner_1[1] + refined_corner_2[1] + refined_center[1])/3;
                    updated_mean_pt[2] = (refined_corner_1[2] + refined_corner_2[2] + refined_center[2])/3;

                    tgf_out[r_i][c_i][s_i].normal = udpated_normal;
                    tgf_out[r_i][c_i][s_i].mean_pt = updated_mean_pt;
                    tgf_out[r_i][c_i][s_i].d = -(udpated_normal.dot(updated_mean_pt));
                    tgf_out[r_i][c_i][s_i].th_dist_d = TH_DIST_ - tgf_out[r_i][c_i][s_i].d;
                    tgf_out[r_i][c_i][s_i].th_outlier_d = -TH_OUTLIER_ - tgf_out[r_i][c_i][s_i].d;

                    tgf_out[r_i][c_i][s_i].node_type = GROUND;
                }
            }
            }
            }

            return;
        };

        void fitTGFWiseTraversableTerrainModel(TriGridField<PointType>& tgf,
                                            std::vector<std::vector<TriGridCorner>>& trigrid_corners,
                                            std::vector<std::vector<TriGridCorner>>& trigrid_centers) {

            updateTGFCornersCenters(trigrid_corners, trigrid_centers);

            revertTraversableNodes(trigrid_corners, trigrid_centers, tgf);
            
            return;
        };

        void segmentNodeGround(const TriGridNode<PointType>& node_in,
                                pcl::PointCloud<PointType>& node_ground_out,
                                pcl::PointCloud<PointType>& node_nonground_out,
                                pcl::PointCloud<PointType>& node_obstacle_out,
                                pcl::PointCloud<PointType>& node_outlier_out) {
            node_ground_out.clear();
            node_nonground_out.clear();
            node_obstacle_out.clear();
            node_outlier_out.clear();

            // segment ground
            Eigen::MatrixXf points(node_in.ptCloud.points.size(),3);
            int j = 0; 
            for (auto& p:node_in.ptCloud.points){
                points.row(j++)<<p.x, p.y, p.z;
            }

            Eigen::VectorXf result = points*node_in.normal;
            for (int r = 0; r<result.rows(); r++){
                if (result[r]<node_in.th_dist_d){
                    if (result[r]<node_in.th_outlier_d){
                        node_outlier_out.push_back(node_in.ptCloud.points[r]);
                    } else {
                        node_ground_out.push_back(node_in.ptCloud.points[r]);
                    }
                } else {
                    node_nonground_out.push_back(node_in.ptCloud.points[r]);
                    if (result[r]<TH_OBSTACLE_HEIGHT_ - node_in.d){
                        node_obstacle_out.push_back(node_in.ptCloud.points[r]);
                        node_obstacle_out.points.back().intensity = result[r] + node_in.d;
                    }
                }
            }

            return;
        }

        void segmentTGFGround(const TriGridField<PointType>& tgf_in, 
                        pcl::PointCloud<PointType>& ground_cloud_out, 
                        pcl::PointCloud<PointType>& nonground_cloud_out,
                        pcl::PointCloud<PointType>& obstacle_cloud_out,
                        pcl::PointCloud<PointType>& outlier_cloud_out) {
            ground_cloud_out.clear();
            nonground_cloud_out.clear();
            obstacle_cloud_out.clear();

            for (int r_i = 0; r_i < rows_; r_i++){
            for (int c_i = 0; c_i < cols_; c_i++){
            for (int s_i = 0; s_i < tgf_in[r_i][c_i].size(); s_i++) {
                if (!tgf_in[r_i][c_i][s_i].is_curr_data) {
                    continue;
                }
                if (tgf_in[r_i][c_i][s_i].node_type == GROUND) {
                    segmentNodeGround(tgf_in[r_i][c_i][s_i], ptCloud_nodewise_ground_, ptCloud_nodewise_nonground_, ptCloud_nodewise_obstacle_, ptCloud_nodewise_outliers_);
                } else {
                    ptCloud_nodewise_nonground_ = tgf_in[r_i][c_i][s_i].ptCloud;
                    ptCloud_nodewise_obstacle_ = tgf_in[r_i][c_i][s_i].ptCloud;
                }
                ground_cloud_out += ptCloud_nodewise_ground_;
                nonground_cloud_out += ptCloud_nodewise_nonground_;
                outlier_cloud_out += ptCloud_nodewise_outliers_;
                obstacle_cloud_out += ptCloud_nodewise_obstacle_;
            }
            }
            }

            return;
        };

        void segmentTGFGround_developing(const TriGridField<PointType>& tgf_in, 
                        pcl::PointCloud<PointType>& ground_cloud_out, 
                        pcl::PointCloud<PointType>& nonground_cloud_out,
                        pcl::PointCloud<PointType>& obstacle_cloud_out,
                        pcl::PointCloud<PointType>& outlier_cloud_out) {
            ground_cloud_out.clear();
            nonground_cloud_out.clear();
            obstacle_cloud_out.clear();
            TriGridIdx curr_tgf_idx;
            std::vector<TriGridIdx> adj_idx_vec;
            pcl::PointCloud<PointType> outlier_tmp;
            outlier_tmp.clear();
            outlier_tmp.reserve(NODEWISE_PTCLOUDSIZE);
            for (int r_i = 0; r_i < rows_; r_i++){
            for (int c_i = 0; c_i < cols_; c_i++){
            for (int s_i = 0; s_i < (int) tgf_in[r_i][c_i].size(); s_i++) {
                if (!tgf_in[r_i][c_i][s_i].is_curr_data) {
                    continue;
                }
                if (tgf_in[r_i][c_i][s_i].node_type == UNKNOWN){
                    continue;
                }
                if (tgf_in[r_i][c_i][s_i].node_type == GROUND) {
                    segmentNodeGround(tgf_in[r_i][c_i][s_i], ptCloud_nodewise_ground_, ptCloud_nodewise_nonground_, ptCloud_nodewise_obstacle_, ptCloud_nodewise_outliers_);
                } else {
                    curr_tgf_idx.row = r_i;
                    curr_tgf_idx.col = c_i;
                    curr_tgf_idx.tri = s_i;
                    
                    searchAdjacentNodes(curr_tgf_idx, adj_idx_vec);
                    if (adj_idx_vec.empty()){
                        ptCloud_nodewise_nonground_ = tgf_in[r_i][c_i][s_i].ptCloud;
                        ptCloud_nodewise_obstacle_ = tgf_in[r_i][c_i][s_i].ptCloud;
                    } else {
                        TriGridIdx highest_adj_tri_idx;
                        double highest_weight = 0;
                        bool is_adjacent = false;
                        for (int adj_i = 0; adj_i<adj_idx_vec.size() ; adj_i++){
                            if (getTriGridNode(adj_idx_vec[adj_i]).weight < TH_WEIGHT_) {
                                continue;
                            }
                            is_adjacent = true;
                            if (getTriGridNode(adj_idx_vec[adj_i]).weight  > highest_weight) {
                                highest_weight = getTriGridNode(adj_idx_vec[adj_i]).weight;
                                highest_adj_tri_idx = adj_idx_vec[adj_i];
                            }
                        }

                        if (is_adjacent){
                            TriGridNode<PointType> tmp_node = getTriGridNode(highest_adj_tri_idx);
                            tmp_node.ptCloud = getTriGridNode(curr_tgf_idx).ptCloud;
                            segmentNodeGround(tmp_node, ptCloud_nodewise_ground_, ptCloud_nodewise_nonground_, ptCloud_nodewise_obstacle_, ptCloud_nodewise_outliers_);
                        } else {
                            ptCloud_nodewise_nonground_ = tgf_in[r_i][c_i][s_i].ptCloud;
                            ptCloud_nodewise_obstacle_ = tgf_in[r_i][c_i][s_i].ptCloud;
                        }
                    }
                }
                ground_cloud_out += ptCloud_nodewise_ground_;
                nonground_cloud_out += ptCloud_nodewise_nonground_;
                obstacle_cloud_out += ptCloud_nodewise_obstacle_;
                outlier_cloud_out += ptCloud_nodewise_outliers_;
            }
            }
            }

            return;
        };        
    };
}
#endif