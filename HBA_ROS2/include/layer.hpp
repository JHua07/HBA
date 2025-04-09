#ifndef LAYER_HPP
#define LAYER_HPP

#include <iostream>
#include <thread>
#include <fstream>
#include <iomanip>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <Eigen/SparseCholesky>
#include <visualization_msgs/msg/marker.h>
#include <visualization_msgs/msg/marker_array.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "tools.hpp"
#include "mypcl.hpp"

#define FULL_HESS
#define WIN_SIZE 10 // 窗口大小，即一个窗口内的位姿数量
#define GAP 5       // 步长

/***********************这个头文件的作用是存放分层的功能块************************/
class LAYER
{
public:
    /************变量定义*****************/
    int pose_size;          // 位姿数量
    int layer_num;          // 层索引
    int max_iter;           // 最大迭代次数
    int part_length;        // 除开最后一个线程，每个线程分配的任务量,也就是每个线程处理的窗口
    int left_size;          // 最后一个线程需要处理的所有位姿数量
    int left_h_size;        //
    int j_upper;            // 最后一个线程剩下的窗口移动次数
    int tail;               // 一层之内所有窗口按步长移动完成后，剩下的不足一次移动步长的位姿数量, 因为如果大于5，那么窗口就能再移动一次, 拿23个位姿举例，WIN_SIZE = 10, GAP = 5, thread_num = 2举例，tail == 3
    int thread_num;         // 线程数量
    int gap_num;            // 满窗口移动次数
    int last_win_size;      // 最后不足一个窗口大小的位姿数量，比如23个点，WIN_SIZE = 10 ,GAP = 5, 此时last_win_size = 8
    int left_gap_num;       // 最后一个线程剩余满窗口任务量(PS: 这里不要陷入误区，因为最后一个线程不一定能分到33个任务)，拿23个位姿举例，WIN_SIZE = 10, GAP = 5, thread_num = 2举例，满足窗口内10个点的移动一共只移动两次，此时最后还剩下3个点（移动后窗口内有8个点），那么就还需要移动一次，以遍历所有的点
    double downsample_size; // 降采样大小
    double voxel_size;      // 体素大小
    double eigen_ratio;     // 特征值比率
    double reject_ratio;    // 拒绝比率

    std::string data_path;          // 数据路径
    vector<mypcl::pose> pose_vec;   // 位姿向量
    std::vector<thread *> mthreads; // 线程向量
    std::vector<double> mem_costs;  // 内存消耗成本

    std::vector<VEC(6)> hessians;                      // Hessian矩阵, 定义6个6*1的向量
    std::vector<pcl::PointCloud<PointType>::Ptr> pcds; // 点云指针

    // 基本参数
    LAYER()
    {
        pose_size = 0;
        layer_num = 1;
        max_iter = 10;
        downsample_size = 0.1;
        voxel_size = 4.0;
        eigen_ratio = 0.1;
        reject_ratio = 0.05;
        pose_vec.clear();
        mthreads.clear();
        pcds.clear();
        hessians.clear();
        mem_costs.clear();
    }

    /// @brief 初始化mthreads, mem_costs, pcds, hessians
    /// @param total_layer_num_ 
    void init_storage(int total_layer_num_)
    {
        mthreads.resize(thread_num);
        mem_costs.resize(thread_num);

        pcds.resize(pose_size);
        pose_vec.resize(pose_size);

#ifdef FULL_HESS
        if(layer_num < total_layer_num_)// 若非全局Ba
        {
            // 计算除开最后一个线程外的Hessian矩阵大小, 即：线程数*（窗口大小-1）*窗口大小/2 * 除开最后一个线程外每个线程的任务量
            // (WIN_SIZE - 1) * WIN_SIZE / 2计算的是一个窗口内的Hessian有多少个hessian矩阵，因为hessian矩阵是两个位姿之间就会存在一个
            int hessian_size = (thread_num - 1) * (WIN_SIZE - 1) * WIN_SIZE / 2 * part_length;
            hessian_size += (WIN_SIZE - 1) * WIN_SIZE / 2 * left_gap_num; // 计算最后一个线程满窗口的的Hessian矩阵数量
            if(tail > 0)//如果存在不足win_size的窗口
            {
                hessian_size += (last_win_size - 1) * last_win_size / 2; // 计算最后一个线程不满窗口的Hessian矩阵数量
            }
            hessians.resize(hessian_size);
            std::cout << "hessian_size:" << hessian_size << endl;
        }
        else// 全局Ba
        {
            int hessian_size = pose_size * (pose_size - 1) / 2;
            hessians.resize(hessian_size);
            std::cout << "hessian_size:" << hessian_size << endl;
        }
#endif
        for(int i = 0; i < thread_num; i++)
        {
            mem_costs.push_back(0);
        }
    }

    /// @brief 初始化pose_size, tail, gap_num, last_win_size, part_length, left_gap_num, left_size, left_h_size, j_upper
    /// @param pose_size_ 
    void init_layer_param(int pose_size_ = 0)
    {
        // 第一层初始化
        if(layer_num == 1)
        {
            pose_size = pose_vec.size();
        }
        else
        {
            pose_size = pose_size_;//自动传入
        }
        tail = (pose_size - WIN_SIZE) % GAP;
        gap_num = (pose_size - WIN_SIZE) / GAP;
        last_win_size = pose_size - GAP * (gap_num + 1);//GAP * (gap_num + 1)是满窗口的位姿数量
        part_length = ceil((gap_num + 1) / double(thread_num));//对于满窗口，每个线程分配处理的窗口数量

        // 如果计算出的任务量超出了总任务量，则使用 floor 函数向下取整，重新计算每个线程的任务量
        if(gap_num - (thread_num - 1) * part_length < 0)
        {
            part_length = floor((gap_num + 1) / double(thread_num));
        }

        // 这个循环确保每个线程分配到的任务量是合理的。如果任务量为0或者分配不均匀(最后一个线程剩下任务超过两倍的part_length)，则减少线程数量并重新计算任务量。
        while (part_length == 0 || (gap_num - (thread_num - 1) * part_length + 1) / double(part_length) > 2)
        {
            thread_num -= 1;
            part_length = ceil((gap_num + 1) / double(thread_num));
            if (gap_num - (thread_num - 1) * part_length < 0)
                part_length = floor((gap_num + 1) / double(thread_num));
        }

        // 最后一个线程剩下的任务量
        left_gap_num = gap_num - (thread_num - 1) * part_length + 1;

        if(tail == 0)//没有不足WIN_SIZE个位姿的情况
        {
            // 最后一个线程需要处理的所有位姿数量，因为此时tail==0
            left_size = left_gap_num * WIN_SIZE;
            // 最后一个线程需要处理的所有Hessian矩阵数量
            left_h_size = (left_gap_num - 1) * GAP + WIN_SIZE - 1;
            // 最后一个线程需要处理的所有窗口数量，包括满窗口和不满窗口
            j_upper = gap_num - (thread_num - 1) * part_length + 1;
        }
        else
        {
            left_size = left_gap_num * WIN_SIZE + GAP + tail;
            left_h_size = left_gap_num * GAP + last_win_size - 1;
            j_upper = gap_num - (thread_num - 1) * part_length + 2;
        }
        std::cout << "Init parameter :" << endl;
        std::cout << "layer_num: " << layer_num << " | thread_num: " << thread_num << " | pose_size: " << pose_size
                  << " | max_iter: " << max_iter << " | part_length: " << part_length
                  << " | gap_num: " << gap_num << " | last_win_size: " << last_win_size
                  << " | left_gap_num: " << left_gap_num << " | tail: " << tail
                  << " | left_size: " << left_size << " | left_h_size: " << left_h_size
                  << " | j_upper: " << j_upper
                  << " | downsample_size: " << downsample_size
                  << " | voxel_size: " << voxel_size
                  << " | eigen_ratio: " << eigen_ratio
                  << " | reject_ratio: " << reject_ratio << std::endl;
    }
};
#endif