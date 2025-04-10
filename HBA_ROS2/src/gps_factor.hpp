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
#include <nav_msgs/msg/odometry.hpp>

#include <mutex>
#include <assert.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
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

#include <GeographicLib/LocalCartesian.hpp>
#include "mypcl.hpp"

using namespace std;
using namespace Eigen;

class GPS_Factor
{
public:
    // GNSS数据(原始)
    double time;
    double latitude;
    double longitude;
    double altitude;
    double local_E;
    double local_N;
    double local_U;
    int status;
    int service;

    // GNSS数据(处理好的转换到IMU坐标系下的坐标)
    struct gps_imu_pose3d
    {
        // 先初始化再定义，确保变量被初始化
        gps_imu_pose3d(Eigen::Vector3d _t = Eigen::Vector3d(0, 0, 0)) : t(_t) {}
        Eigen::Vector3d t;
    };

    std::vector<double> gps_time; // GPS时间

    GPS_Factor();
    ~GPS_Factor();
    std::vector<gps_imu_pose3d> read_gps_imu_data(std::string filename, Eigen::Vector3d te);
    void Add_GPS_Factor(std::vector<gps_imu_pose3d> gps_pose, std::vector<mypcl::pose> lio_pose, std::vector<double> lidar_time, gtsam::NonlinearFactorGraph graph);
    float pointDistance(Eigen::Vector3d p1, Eigen::Vector3d p2);

private:
};

std::vector<GPS_Factor::gps_imu_pose3d> GPS_Factor::read_gps_imu_data(std::string filename, Eigen::Vector3d te = Eigen::Vector3d(0, 0, 0))
{
    std::vector<gps_imu_pose3d> gps_pose;
    std::fstream file;
    file.open(filename);
    double gps_t,tx,ty,tz;

    while(file >> gps_t >> tx >> ty >>tz)
    {
        Eigen::Vector3d t(tx, ty, tz);
        gps_pose.push_back(gps_imu_pose3d(t + te));
        gps_time.push_back(gps_t); // 读取GPS时间
    }
    file.close();
    return gps_pose;
}

void GPS_Factor::Add_GPS_Factor(std::vector<gps_imu_pose3d> gps_pose, std::vector<mypcl::pose> lio_pose, std::vector<double> lidar_time, gtsam::NonlinearFactorGraph graph)
{
    if(gps_pose.empty() || lidar_time.empty() || lio_pose.empty())
    {
        std::cout << "****some data is empty, please check your datas!****" << std::endl;
        return;
    }
    std::vector<int> index_gps2lidar;
    index_gps2lidar.resize(gps_pose.size());
    int k = 0;
    // lidar_time.size() == lio_pose.size()
    // 解释一下，例如 j = index_gps2lidar[k]， 相当于第k个GPS点对应的lio的位姿索引为j
    for(int i = 0; i < lidar_time.size(); i++)
    {
        // 需要寻找到与当前激光点云时间最接近的GPS时间
        if(fabs(lidar_time[i] - gps_time[index_gps2lidar[k]]) > 0.05)
        {
            k++;
            continue;
        }
        else
        {
            index_gps2lidar[k] = i;
            k++;
        }
    }

    float noise_x = 0.0025; //  x 方向的协方差
    float noise_y = 0.0025;
    float noise_z = 0.0025;
    for (int i = 0; i < gps_pose.size(); i++)
    {
        // 若当前处理的位姿与第一个位姿的距离小于5m，则跳过
        if (pointDistance(lio_pose.front().t, lio_pose[index_gps2lidar[i]].t) < 5.0)
        {
            continue;
        }
        if (abs(gps_pose[i].t.x()) < 1e-6 && abs(gps_pose[i].t.y()) < 1e-6)
        {
            std::cout << "****Invalid GPS data****" << std::endl;
            continue;
        }
        gtsam::Vector3 Vector3(3);
        Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
        gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Diagonal::Variances(Vector3);
        gtsam::GPSFactor gps_factor(index_gps2lidar[i], gtsam::Point3(gps_pose[i].t.x(), gps_pose[i].t.y(), gps_pose[i].t.z()), gps_noise);
        graph.add(gps_factor);
    }
    std::cout << "****GPS factor add complete!****" << std::endl;
}

float pointDistance(Eigen::Vector3d p1, Eigen::Vector3d p2)
{
    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]) + (p1[2] - p2[2]) * (p1[2] - p2[2]));
}