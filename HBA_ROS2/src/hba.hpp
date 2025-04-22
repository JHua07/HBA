#ifndef HBA_HPP
#define HBA_HPP

#include <iostream>
#include <thread>
#include <fstream>
#include <iomanip>

#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <Eigen/SparseCholesky>
#include <Eigen/StdVector>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.h>
#include <visualization_msgs/msg/marker_array.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/pose_array.h>
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>
#include <assert.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "tools.hpp"
#include "layer.hpp"
#include "mypcl.hpp"
#include "voxel.hpp"
#include "gps_factor.hpp"

using namespace std;
using namespace Eigen;

int pcd_name_fill_num = 0;
int total_layer_num, thread_num; // 总层数量，线程数量
string data_path;
bool enable_gps_factor = true; // 是否启用GPS因子
bool gps_imu_info = false; // 是否启用GPS外参
class HBA
{
public:
    int thread_num, total_layer_num;
    std::vector<LAYER> layers;
    std::string data_path;

    // 初始化图层状态, 主要作用是传入底层的位姿以及更新后续层的位姿
    HBA(int total_layer_num_, std::string data_path_, int thread_num_, GPS_Factor &gps_factor_func);

    void update_next_layer_state(int cur_layer_num);

    // 使用GTSAM进行位姿图优化
    void pose_graph_optimization(GPS_Factor &gps_factor_func);
};
void parallel_compute_tool(LAYER &layer, int thread_id, LAYER &next_layer, int i, int win_size);

void cut_voxel(unordered_map<VOXEL_LOC, OCTO_TREE_ROOT *> &feat_map, pcl::PointCloud<PointType> &feat_pt, Eigen::Quaterniond q, Eigen::Vector3d t, int fnum, double voxel_size, int window_size, float eigen_ratio);
void parallel_head(LAYER &layer, int thread_id, LAYER &next_layer);
void parallel_tail(LAYER &layer, int thread_id, LAYER &next_layer);
void global_ba(LAYER &layer);
void distribute_thread(LAYER &layer, LAYER &next_layer);
void interpolate_pose(std::vector<mypcl::pose> &pose_vec_orig, std::vector<mypcl::pose> &pose_vec_tran, std::vector<double> gps_time, std::vector<double> lidar_time);

#endif