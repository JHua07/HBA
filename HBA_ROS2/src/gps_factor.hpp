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

// 记录GPS插值后，对应的lio的索引，例如，对于第k个GPS点，在插值之后的pose_vec_tran中，该点的索引为index_interpolate[k]
std::vector<int> index_interpolate;

class GPS_Factor
{
public:
    // GNSS数据(原始)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double time;
    std::vector<double> latitude;
    std::vector<double> longitude;
    std::vector<double> altitude;
    std::vector<double> local_E;
    std::vector<double> local_N;
    std::vector<double> local_U;

    double origin_longitude;
    double origin_latitude;
    double origin_altitude;

    // 杆臂偏移
    double extrinsic_T[3] = {0.0, 0.0, 0.0};

    std::vector<Eigen::Vector3d> pose_cov;

    double cov_threshold = 25; // 协方差阈值

    // GNSS数据(处理好的转换到IMU坐标系下的坐标)
    struct gps_imu_pose3d
    {
        // 先初始化再定义，确保变量被初始化
        gps_imu_pose3d(Eigen::Vector3d _t = Eigen::Vector3d(0, 0, 0)) : t(_t) {}
        Eigen::Vector3d t;
    };

    std::vector<double> gps_time_raw;                      // GPS原始时间
    std::vector<double> gps_time;                          // 能够有匹配点的GPS时间
    std::vector<GPS_Factor::gps_imu_pose3d> enu_pose;      // ENU坐标系下GPS坐标
    std::vector<GPS_Factor::gps_imu_pose3d> gps_pose_tran; // 转换到IMU坐标系下的GPS坐标
    std::vector<gps_imu_pose3d> read_gps_imu_data(std::string filename, Eigen::Vector3d te);
    void Add_GPS_Factor(std::vector<gps_imu_pose3d> gps_pose, std::vector<mypcl::pose> lio_pose, gtsam::NonlinearFactorGraph graph, std::vector<VEC(6)> init_cov, std::vector<int> index_interpolate);
    double pointDistance(Eigen::Vector3d p1, Eigen::Vector3d p2);

    // 读取输入的GPS原始信息并转换到ENU坐标系下,后利用轨迹匹配转换到IMU坐标系下
    std::vector<GPS_Factor::gps_imu_pose3d> read_gps_raw_info(std::string filename, std::string data_path);

    void InitOriginPosition(double lat, double lon, double alt);
    void UpdateXYZ(double lat, double lon, double alt);

    void Reverse(
        const double &local_E, const double &local_N, const double &local_U,
        double &lat, double &lon, double &alt);

    void path_match(std::vector<mypcl::pose> lio_pose, std::vector<double> local_E, std::vector<double> local_N, std::vector<double> local_U, std::vector<double> gps_time);

private:
    GeographicLib::LocalCartesian geo_converter;
    Eigen::Vector3d Gnss_T_wrt_Lidar;
    Eigen::Matrix3d Gnss_R_wrt_Lidar;
};

std::vector<GPS_Factor::gps_imu_pose3d> GPS_Factor::read_gps_imu_data(std::string filename, Eigen::Vector3d te = Eigen::Vector3d(0, 0, 0))
{
    std::vector<gps_imu_pose3d> gps_pose;
    std::fstream file;
    file.open(filename);
    double gps_t, tx, ty, tz;

    while (file >> gps_t >> tx >> ty >> tz)
    {
        Eigen::Vector3d t(tx, ty, tz);
        gps_pose.push_back(gps_imu_pose3d(t + te));
        gps_time.push_back(gps_t); // 读取GPS时间
    }
    file.close();
    return gps_pose;
}

void GPS_Factor::Add_GPS_Factor(std::vector<gps_imu_pose3d> gps_pose, std::vector<mypcl::pose> lio_pose, gtsam::NonlinearFactorGraph graph, std::vector<VEC(6)> init_cov, std::vector<int> index_interpolate)
{
    if (gps_pose.empty() || lidar_time.empty() || lio_pose.empty())
    {
        std::cout << "****some data is empty, please check your datas!****" << std::endl;
        if (gps_pose.empty())
        {
            std::cout << "****  GPS Pose Empty !!!   ****" << std::endl;
        }
        if (lidar_time.empty())
        {
            std::cout << "****  Lidar Time Empty !!!   ****" << std::endl;
        }
        if (lio_pose.empty())
        {
            std::cout << "****  LIO Pose Empty !!!   ****" << std::endl;
        }
        return;
    }

    float noise_x = 0.0025; //  x 方向的协方差
    float noise_y = 0.0025;
    float noise_z = 0.0025;

    mypcl::pose last_lio_pose = lio_pose.front(); // 上一个里程计位姿
    for (int i = 0; i < gps_pose.size(); i++)
    {
        /* 感觉这里没必要，因为如果是闭环的话，可能会出现首尾相连的情况，而会忽略中间的情况
        // 若首尾里程计位姿的距离小于5m，则跳过
        if (pointDistance(lio_pose.front().t, lio_pose[index_gps2lidar[i]].t) < 5.0)
        {
            continue;
        }
        last_lio_pose = lio_pose[index_gps2lidar[i]];
        */
        // 更新上一个里程计位姿

        if (abs(gps_pose[i].t.x()) < 1e-6 && abs(gps_pose[i].t.y()) < 1e-6)
        {
            std::cout << "****Invalid GPS data****" << std::endl;
            continue;
        }

        /*
        if(init_cov[index_interpolate[i]](0) < cov_threshold || init_cov[index_interpolate[i]](1) < cov_threshold)
        {
            std::cout << "****Input datas's cov is small enough, skip add no." << i << "gps factor****" << std::endl;
            continue;
        }
        */
        // 每隔5m添加一个GPS因子
        //std::cout << index_interpolate.size() << std::endl;
        if (pointDistance(last_lio_pose.t, lio_pose[index_interpolate[i]].t) < 5.0)
        {
            continue;
        }
        else
        {
            last_lio_pose = lio_pose[index_interpolate[i]];
        }

        gtsam::Vector3 Vector3(3);
        Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
        gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Diagonal::Variances(Vector3);
        gtsam::GPSFactor gps_factor(index_interpolate[i], gtsam::Point3(gps_pose[i].t.x(), gps_pose[i].t.y(), gps_pose[i].t.z()), gps_noise);
        graph.add(gps_factor);
        std::cout << "" << "Add GPS factor no." << i << " to graph!" << std::endl;
    }
    std::cout << "****GPS factor add complete!****" << std::endl;
}

double GPS_Factor::pointDistance(Eigen::Vector3d p1, Eigen::Vector3d p2)
{
    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]) + (p1[2] - p2[2]) * (p1[2] - p2[2]));
}

// 初始化原点， WGS84 -> ENU   ???  调试结果好像是 NED 北东地
void GPS_Factor::InitOriginPosition(double lat, double lon, double alt)
{
    geo_converter.Reset(lat, lon, alt);
    std::cout << "Init   Gnss  OriginPosition " << std::endl;
    origin_latitude = lat;
    origin_longitude = lon;
    origin_altitude = alt;

    // 存储原点经纬度
    // latitude.push_back(lat);
    // longitude.push_back(lon);
    // altitude.push_back(alt);
}

// 获取更新后的ENU坐标
void GPS_Factor::UpdateXYZ(double lat, double lon, double alt)
{
    double local_E_raw, local_N_raw, local_U_raw;
    geo_converter.Forward(lat, lon, alt, local_E_raw, local_N_raw, local_U_raw);
    // 存储更新后的坐标
    local_E.push_back(local_E_raw);
    local_N.push_back(local_N_raw);
    local_U.push_back(-local_U_raw);
}

void GPS_Factor::Reverse(
    const double &local_E, const double &local_N, const double &local_U,
    double &lat, double &lon, double &alt)
{
    geo_converter.Reverse(local_E, local_N, local_U, lat, lon, alt);
}

std::vector<GPS_Factor::gps_imu_pose3d> GPS_Factor::read_gps_raw_info(std::string filename, std::string data_path)
{
    std::fstream file;
    file.open(filename);
    double time;
    Eigen::Vector3d pose_cov_src;
    double lat, lon, alt;
    bool gps_init = false;

    while (file >> time >> lat >> lon >> alt >> pose_cov_src(0) >> pose_cov_src(1) >> pose_cov_src(2))
    {
        // 读取GPS时间
        gps_time_raw.push_back(time);
        // std::cout << "gps_time:" << time << endl;
        //  读取协方差
        pose_cov.push_back(pose_cov_src);
        //  初始化位置
        if (!gps_init)
        {
            InitOriginPosition(lat, lon, alt);
            gps_init = true;
            UpdateXYZ(lat, lon, alt);
        }
        else
        {
            UpdateXYZ(lat, lon, alt);
        }
    }
    file.close();
    std::cout << "****GPS data read complete!****" << std::endl;
    // 此时完成了GPS点的读取和转换，现在的坐标在ENU坐标系下,需要转换到IMU坐标系下
    // 先读取lio_pose
    std::vector<mypcl::pose> lio_pose_orig = mypcl::read_pose(data_path + "pose.json");
    std::cout << "****LIO data read complete!****" << std::endl;

    path_match(lio_pose_orig, local_E, local_N, local_U, gps_time_raw);
    std::cout << "****GPS data path match complete!****" << std::endl;

    // 输出Gnss_R_wrt_Lidar
    std::cout << "Gnss_R_wrt_Lidar = \n"
              << Gnss_R_wrt_Lidar << std::endl;

    gps_pose_tran.resize(enu_pose.size());
    std::cout << "Start transform enu coordinations to IMU axis!" << std::endl;

    std::vector<GPS_Factor::gps_imu_pose3d> gps_pose_trans_xyz;
    gps_pose_trans_xyz.reserve(enu_pose.size());
    for(int i = 0; i < enu_pose.size(); i++)
    {
        gps_pose_trans_xyz[i].t = Gnss_R_wrt_Lidar * enu_pose[i].t + Gnss_T_wrt_Lidar;
    }
    for(int i = 0 ; i < enu_pose.size(); i++)
    {
        gps_pose_tran[i].t[0] = gps_pose_trans_xyz[i].t[2];
        gps_pose_tran[i].t[1] = gps_pose_trans_xyz[i].t[1];
        gps_pose_tran[i].t[2] = gps_pose_trans_xyz[i].t[0];
    }
    /*
    for (int i = 0; i < enu_pose.size(); i++)
    {
        gps_pose_tran[i].t = Gnss_R_wrt_Lidar * enu_pose[i].t + Gnss_T_wrt_Lidar;
    }*/
    return gps_pose_tran;
}

void GPS_Factor::path_match(std::vector<mypcl::pose> lio_pose, std::vector<double> local_E, std::vector<double> local_N, std::vector<double> local_U, std::vector<double> gps_time_raw)
{
    // 找到gps_point时间最近的lio_pose的索引
    std::vector<int> index_gps2lidar;
    int k = 0;
    // lidar_time.size() == lio_pose.size()
    // 解释一下，例如 j = index_gps2lidar[k]， 相当于第k个GPS点对应的lio的位姿索引为j
    std::cout << "lidar_time_size:" << lidar_time.size() << std::endl;
    for (int i = 0; i < lidar_time.size(); i++)
    {
        // std::cout << "lidar_time[" << i << "]:" << lidar_time[i] << std::endl;
        //  需要寻找到与当前激光点云时间最接近的GPS时间
        for (int j = k; j < gps_time_raw.size(); j++)
        {
            if (fabs(lidar_time[i] - gps_time_raw[j]) < 0.06 && gps_time_raw[j] < lidar_time.back())
            {
                // std::cout << "no." << i + 1 << "lidar_time = " << fixed << setprecision(6) << lidar_time[i]  << endl;
                // std::cout << "no." << j + 1 << "gps_time = " << fixed << setprecision(6)<< gps_time_raw[j]  << endl;
                index_gps2lidar.push_back(i);
                enu_pose.push_back(Eigen::Vector3d(local_E[i], local_N[i], local_U[i]));
                gps_time.push_back(gps_time_raw[j]);
                k++;
                /*
                if(index_gps2lidar[j] == 0 && j > 0)
                {
                    index_gps2lidar.pop_back();
                    enu_pose.pop_back();
                }
                */
                break;
            }
        }
    }

    std::cout << "index_gps2lidar size:" << index_gps2lidar.size() << std::endl;
    /*
    for(int i = 0; i < index_gps2lidar.size(); i++)
    {
        std::cout << "index_gps2lidar[" << i << "]:" << index_gps2lidar[i] << std::endl;
    }
    */
    k = 0;
    // 找到索引之后，进行线性插值, 内插计算在gps有点的时候，对应时刻的lio的位姿
    std::vector<mypcl::pose> gps_pose_in_liopath;
    for (int j = 0; j < lidar_time.size(); j++)
    {
        // 如果找的到索引，说明此时需要对GPS点进行插值
        if (j == index_gps2lidar[k])
        {
            mypcl::pose pose_temp;
            if (gps_time[k] >= lidar_time[index_gps2lidar[k]])
            {
                pose_temp.t = lio_pose[j].t + (gps_time[k] - lidar_time[j]) * (lio_pose[j + 1].t - lio_pose[j].t) / (lidar_time[j + 1] - lidar_time[j]);
                pose_temp.q = lio_pose[j].q.slerp((gps_time[k] - lidar_time[j]) / (lidar_time[j + 1] - lidar_time[j]), lio_pose[j + 1].q);
                gps_pose_in_liopath.push_back(pose_temp);
            }
            else
            {
                pose_temp.t = lio_pose[j - 1].t + (gps_time[k] - lidar_time[j - 1]) * (lio_pose[j].t - lio_pose[j - 1].t) / (lidar_time[j] - lidar_time[j - 1]);
                pose_temp.q = lio_pose[j - 1].q.slerp((gps_time[k] - lidar_time[j - 1]) / (lidar_time[j] - lidar_time[j - 1]), lio_pose[j].q);
                gps_pose_in_liopath.push_back(pose_temp);
            }
            k++;
        }
    }
    std::cout << " Interpolate GPS Point to LioPath, Size:" << gps_pose_in_liopath.size() << std::endl;
    std::cout << "ENU_size:" << enu_pose.size() << std::endl;
    /*
    for(int i = 0; i < gps_pose_in_liopath.size(); i++)
    {
        // 先将ENU坐标存到一个pose_vec中，方便后续使用
        enu_pose[i].t = Eigen::Vector3d(local_E[i], local_N[i], local_U[i]);
        //enu_pose[i].t[0] = local_E[i];
        //enu_pose[i].t[1] = local_N[i];
        //enu_pose[i].t[2] = local_U[i];
    }
    */
    // 此时enu_pose和gps_pose_in_liopath的点是一一对应的, 开始轨迹匹配
    // 1. 计算质心坐标
    Eigen::Vector3d enu_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d lio_centroid = Eigen::Vector3d::Zero();
    size_t N = enu_pose.size();
    for (const auto &p : enu_pose)
    {
        enu_centroid += p.t;
    }
    for (const auto &p : lio_pose)
    {
        lio_centroid += p.t;
    }
    enu_centroid /= N;
    lio_centroid /= N;

    // 2. 去中心化并计算协方差矩阵
    Eigen::Matrix3d CovMat = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < N; i++)
    {
        Eigen::Vector3d enu_decent = enu_pose[i].t - enu_centroid;
        Eigen::Vector3d gps_decent = gps_pose_in_liopath[i].t - lio_centroid;
        CovMat += enu_decent * gps_decent.transpose();
    }

    // 3. SVD分解
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(CovMat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
    // 处理反射情况
    if (R.determinant() < 0)
    {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1;
        R = V * svd.matrixU().transpose();
    }

    // 4. 计算平移
    Eigen::Vector3d t = lio_centroid - R * enu_centroid;

    // 更新旋转矩阵
    Gnss_R_wrt_Lidar = R;
    // 更新平移向量
    Gnss_T_wrt_Lidar = t;
}