#ifndef MYPCL_HPP
#define MYPCL_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iomanip>

// 定义四元数和三维向量的变量
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> vector_vec3d;
typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> vector_quad;

// 定义点云类型，默认存储点云的类型为 pcl::PointXYZI
// typedef pcl::PointXYZINormal PointType;
// typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZI PointType;

// 定义一个 6x6 的矩阵
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

// 定义激光雷达的时间戳
std::vector<double> lidar_time;
std::vector<double> timestamp;
bool flag_lidar_time = false;

namespace mypcl
{
    // 定义用于存储位姿的数据类型结构
    struct pose
    {
        // 先初始化再定义，确保变量被初始化
        pose(Eigen::Quaterniond _q = Eigen::Quaterniond(1, 0, 0, 0),
             Eigen::Vector3d _t = Eigen::Vector3d(0, 0, 0)) : q(_q), t(_t) {}
        Eigen::Quaterniond q;
        Eigen::Vector3d t;
    };

    // 读取pcd文件, 输入文件路径，pcd文件名，点云指针，文件编号，文件前缀
    void loadPCD(std::string filepath, int pcd_fill_num, pcl::PointCloud<PointType>::Ptr &pc, int num, std::string prefix = "")
    {
        std::stringstream ss;
        // 设置读取文件名的前缀，比如文件名是0001.pcd，那么这个填充数就是4, launch文件中设置
        if(pcd_fill_num > 0)
        {
            ss << std::setw(pcd_fill_num) << std::setfill('0') << num;
        }
        else
        {
            ss << num;
        }
        pcl::io::loadPCDFile(filepath + prefix + ss.str() + ".pcd", *pc);
    }

    // 保存pcd文件
    void savePCD(std::string filePath, int pcd_fill_num, pcl::PointCloud<PointType>::Ptr &pc, int num, std::string prefix = "")
    {
        std::stringstream ss;
        if (pcd_fill_num > 0)
        {
            ss << std::setw(pcd_fill_num) << std::setfill('0') << num;
        }
        else
        {
            ss << num;
        }
        pcl::io::savePCDFileBinary(filePath + prefix + ss.str() + ".pcd", *pc);
    }

    // 读取位姿文件, 输入文件名，四元数，三维向量
    std::vector<pose> read_pose(std::string filename, Eigen::Quaterniond qe = Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d te = Eigen::Vector3d(0, 0, 0))
    {
        std::vector<pose> pose_vec;
        std::fstream file;
        file.open(filename);
        double lt, tx, ty, tz, w, x, y, z;

        while(file >> lt >> tx >> ty >> tz >> x >> y >> z >> w) // 是否达到文件流的末尾
        {
            Eigen::Quaterniond q(w, x, y, z);
            Eigen::Vector3d t(tx, ty, tz);

            //位姿全部转换到第一个点云坐标系下
            pose_vec.push_back(pose(qe * q, qe * t + te));
            if(!flag_lidar_time)
            {
                lidar_time.push_back(lt);
            }

        }
        flag_lidar_time = true;
        file.close();
        return pose_vec;
    }

    // 位姿变换函数，输入点云pc_in，变换后的点云pt_out，位姿t和q
    void transform_pointcloud(pcl::PointCloud<PointType> const &pc_in,
                              pcl::PointCloud<PointType> &pt_out,
                              Eigen::Vector3d t,
                              Eigen::Quaterniond q)
    {
        size_t size = pc_in.points.size();
        pt_out.points.resize(size);
        for (size_t i = 0; i < size; i++)
        {
            Eigen::Vector3d pt_cur(pc_in.points[i].x, pc_in.points[i].y, pc_in.points[i].z);
            Eigen::Vector3d pt_to;
            // if(pt_cur.norm()<0.3) continue;
            pt_to = q * pt_cur + t;
            pt_out.points[i].x = pt_to.x();
            pt_out.points[i].y = pt_to.y();
            pt_out.points[i].z = pt_to.z();
            pt_out.points[i].intensity = pc_in.points[i].intensity;
            // pt_out.points[i].r = pc_in.points[i].r;
            // pt_out.points[i].g = pc_in.points[i].g;
            // pt_out.points[i].b = pc_in.points[i].b;
        }
    }

    // 合并点云, 将所有点云合并到第一个点云中,pc1是指针，可以对第一个点云进行数据的操作，pc2是只读点云, 最后返回合并后的点云
    pcl::PointCloud<PointType>::Ptr append_cloud(pcl::PointCloud<PointType>::Ptr pc1, pcl::PointCloud<PointType> pc2)
    {
        size_t size1 = pc1->points.size();
        size_t size2 = pc2.points.size();
        pc1->points.resize(size1 + size2);
        for(size_t i = size1; i < size1 + size2; i++)
        {
            pc1->points[i].x =  pc2.points[i - size1].x;
            pc1->points[i].y =  pc2.points[i - size1].y;
            pc1->points[i].z =  pc2.points[i - size1].z;
            pc1->points[i].intensity = pc2.points[i - size1].intensity;
            // pc1->points[i].r = pc2.points[i-size1].r;
            // pc1->points[i].g = pc2.points[i-size1].g;
            // pc1->points[i].b = pc2.points[i-size1].b;
        }
        return pc1;
    }

    // 写入位姿文件，这个位姿是相对于第一个点云的位姿
    void write_pose(std::vector<pose> &pose_vec, std::string path)
    {
        std::ofstream file;
        file.open(path + "pose_trans.json", std::ofstream::trunc);
        file.close();
        Eigen::Quaterniond q0(pose_vec[0].q.w(), pose_vec[0].q.x(), pose_vec[0].q.y(), pose_vec[0].q.z());
        Eigen::Vector3d t0(pose_vec[0].t(0), pose_vec[0].t(1), pose_vec[0].t(2));
        file.open(path + "pose_trans.json", std::ofstream::app);

        for (size_t i = 0; i < pose_vec.size(); i++)
        {

            // 位姿全部转换到第一个点云坐标系下
            //pose_vec[i].t << q0.inverse()*(pose_vec[i].t-t0);
            /*
            pose_vec[i].q.w() = (q0.inverse() * pose_vec[i].q).w();
            pose_vec[i].q.x() = (q0.inverse() * pose_vec[i].q).x();
            pose_vec[i].q.y() = (q0.inverse() * pose_vec[i].q).y();
            pose_vec[i].q.z() = (q0.inverse() * pose_vec[i].q).z();
            file << pose_vec[i].t(0) << " "
                 << pose_vec[i].t(1) << " "
                 << pose_vec[i].t(2) << " "
                 << pose_vec[i].q.x() << " " << pose_vec[i].q.y() << " "
                 << pose_vec[i].q.z() << " " << pose_vec[i].q.w();
            if (i < pose_vec.size() - 1)
                file << "\n";
            */
            
            file << fixed << setprecision(6)
                 << lidar_time[i] << " "
                 << pose_vec[i].t(0) << " "
                 << pose_vec[i].t(1) << " "
                 << pose_vec[i].t(2) << " "
                 << pose_vec[i].q.x() << " " << pose_vec[i].q.y() << " "
                 << pose_vec[i].q.z() << " " << pose_vec[i].q.w();
             if (i < pose_vec.size() - 1)
                 file << "\n";
        }
        file.close();
        //lidar_time.clear();
    }

    void write_interpolate_pose(std::vector<pose> &pose_vec, std::string path)
    {
        std::ofstream file;
        file.open(path + "pose_inter.json", std::ofstream::trunc);
        file.close();
        Eigen::Quaterniond q0(pose_vec[0].q.w(), pose_vec[0].q.x(), pose_vec[0].q.y(), pose_vec[0].q.z());
        Eigen::Vector3d t0(pose_vec[0].t(0), pose_vec[0].t(1), pose_vec[0].t(2));
        file.open(path + "pose_inter.json", std::ofstream::app);

        for (size_t i = 0; i < pose_vec.size(); i++)
        {

            // 位姿全部转换到第一个点云坐标系下
            //pose_vec[i].t << q0.inverse()*(pose_vec[i].t-t0);
            /*
            pose_vec[i].q.w() = (q0.inverse() * pose_vec[i].q).w();
            pose_vec[i].q.x() = (q0.inverse() * pose_vec[i].q).x();
            pose_vec[i].q.y() = (q0.inverse() * pose_vec[i].q).y();
            pose_vec[i].q.z() = (q0.inverse() * pose_vec[i].q).z();
            file << pose_vec[i].t(0) << " "
                 << pose_vec[i].t(1) << " "
                 << pose_vec[i].t(2) << " "
                 << pose_vec[i].q.x() << " " << pose_vec[i].q.y() << " "
                 << pose_vec[i].q.z() << " " << pose_vec[i].q.w();
            if (i < pose_vec.size() - 1)
                file << "\n";
            */
            
            file << fixed << setprecision(6)
                << timestamp[i] << " "
                << pose_vec[i].t(0) << " "
                << pose_vec[i].t(1) << " "
                << pose_vec[i].t(2) << " "
                << pose_vec[i].q.x() << " " << pose_vec[i].q.y() << " "
                << pose_vec[i].q.z() << " " << pose_vec[i].q.w();
            if (i < pose_vec.size() - 1)
                file << "\n";
        }
        
        file.close();
        //lidar_time.clear();
    }

    void write_evo_pose(std::vector<pose> &pose_vec, std::string path)
    {
        std::ofstream file;
        file.open(path + "pose_trans_evo.txt", std::ofstream::trunc);
        file.close();
        Eigen::Quaterniond q0(pose_vec[0].q.w(), pose_vec[0].q.x(), pose_vec[0].q.y(), pose_vec[0].q.z());
        Eigen::Vector3d t0(pose_vec[0].t(0), pose_vec[0].t(1), pose_vec[0].t(2));
        file.open(path + "pose_trans_evo.txt", std::ofstream::app);

        for (size_t i = 0; i < pose_vec.size(); i++)
        {   
            /*
            // 位姿全部转换到第一个点云坐标系下
            //pose_vec[i].t << q0.inverse()*(pose_vec[i].t-t0);
            pose_vec[i].q.w() = (q0.inverse() * pose_vec[i].q).w();
            pose_vec[i].q.x() = (q0.inverse() * pose_vec[i].q).x();
            pose_vec[i].q.y() = (q0.inverse() * pose_vec[i].q).y();
            pose_vec[i].q.z() = (q0.inverse() * pose_vec[i].q).z();
            file << pose_vec[i].t(0) << " "
                 << pose_vec[i].t(1) << " "
                 << pose_vec[i].t(2) << " "
                 << pose_vec[i].q.x() << " " << pose_vec[i].q.y() << " "
                 << pose_vec[i].q.z() << " " << pose_vec[i].q.w();
            if (i < pose_vec.size() - 1)
                file << "\n";
            */
           file << fixed << setprecision(6)
                << timestamp[i] << " "
                << pose_vec[i].t(0) << " "
                << pose_vec[i].t(1) << " "
                << pose_vec[i].t(2) << " "
                << pose_vec[i].q.x() << " " << pose_vec[i].q.y() << " "
                << pose_vec[i].q.z() << " " << pose_vec[i].q.w();
            if (i < pose_vec.size() - 1)
                file << "\n";
        }
        file.close();
    }
}
#endif