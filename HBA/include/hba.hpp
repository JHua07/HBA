#ifndef HBA_HPP
#define HBA_HPP

#include <thread>
#include <fstream>
#include <iomanip>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <Eigen/SparseCholesky>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "mypcl.hpp"
#include "tools.hpp"
#include "ba.hpp"

class LAYER
{
public:
  /*
  tail:（位姿数-一个窗口大小）% 步长 这个不知道什么意思，第一个窗口按固定步长移动后，最后剩余位姿数量
  gap_num：（位姿数-一个窗口大小）/ 步长， 可能是，窗口移动的次数
  last_win_size:位姿数-步长*（窗口移动次数 + 1）最后一个窗口内的位姿数量
  part_length：（每个线程分配的窗口移动的次数）向上取整，如果（gap_num - (thread_num - 1）*向上取整的part_length) < 0那么，就向下取整，（如果计算出的任务量超出了总任务量，则使用 floor 函数向下取整，重新计算每个线程的任务量）

  */
  int pose_size, layer_num, max_iter, part_length, left_size, left_h_size, j_upper, tail, thread_num,
      gap_num, last_win_size, left_gap_num;
  double downsample_size, voxel_size, eigen_ratio, reject_ratio;

  std::string data_path;
  vector<mypcl::pose> pose_vec;
  std::vector<thread *> mthreads;
  std::vector<double> mem_costs;

  std::vector<VEC(6)> hessians;
  std::vector<pcl::PointCloud<PointType>::Ptr> pcds;

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

  void init_storage(int total_layer_num_)
  {
    mthreads.resize(thread_num);
    mem_costs.resize(thread_num);

    pcds.resize(pose_size);
    pose_vec.resize(pose_size);

#ifdef FULL_HESS
    if (layer_num < total_layer_num_) // 非顶层
    {
      int hessian_size = (thread_num - 1) * (WIN_SIZE - 1) * WIN_SIZE / 2 * part_length;
      hessian_size += (WIN_SIZE - 1) * WIN_SIZE / 2 * left_gap_num;
      if (tail > 0)
        hessian_size += (last_win_size - 1) * last_win_size / 2;
      hessians.resize(hessian_size);
      printf("hessian_size: %d\n", hessian_size);
    }
    else // 顶层
    {
      int hessian_size = pose_size * (pose_size - 1) / 2;
      hessians.resize(hessian_size);
      printf("hessian_size: %d\n", hessian_size);
    }
#endif

    for (int i = 0; i < thread_num; i++)
      mem_costs.push_back(0);
  }

  void init_parameter(int pose_size_ = 0)
  {
    if (layer_num == 1)
      pose_size = pose_vec.size();
    else
      pose_size = pose_size_;
    tail = (pose_size - WIN_SIZE) % GAP;
    gap_num = (pose_size - WIN_SIZE) / GAP;
    last_win_size = pose_size - GAP * (gap_num + 1);
    part_length = ceil((gap_num + 1) / double(thread_num));

    // 如果计算出的任务量超出了总任务量，则使用 floor 函数向下取整，重新计算每个线程的任务量
    if (gap_num - (thread_num - 1) * part_length < 0)
      part_length = floor((gap_num + 1) / double(thread_num));

    // 这个循环确保每个线程分配到的任务量是合理的。如果任务量为0或者分配不均匀，则减少线程数量并重新计算任务量。
    while (part_length == 0 || (gap_num - (thread_num - 1) * part_length + 1) / double(part_length) > 2)
    {
      thread_num -= 1;
      part_length = ceil((gap_num + 1) / double(thread_num));
      if (gap_num - (thread_num - 1) * part_length < 0)
        part_length = floor((gap_num + 1) / double(thread_num));
    }
    // 最后一个线程剩下的任务量
    left_gap_num = gap_num - (thread_num - 1) * part_length + 1;

    if (tail == 0) // 如果没有剩下的位姿
    {
      // 最后一个线程剩下的位姿数量
      left_size = (gap_num - (thread_num - 1) * part_length + 1) * WIN_SIZE;
      // 最后一个线程剩下的hessian数量??
      left_h_size = (gap_num - (thread_num - 1) * part_length) * GAP + WIN_SIZE - 1;
      // 最后一个线程剩下的窗口移动次数??
      j_upper = gap_num - (thread_num - 1) * part_length + 1;
    }
    else
    {
      left_size = (gap_num - (thread_num - 1) * part_length + 1) * WIN_SIZE + GAP + tail;
      left_h_size = (gap_num - (thread_num - 1) * part_length + 1) * GAP + last_win_size - 1;
      j_upper = gap_num - (thread_num - 1) * part_length + 2;
    }

    printf("init parameter:\n");
    printf("layer_num %d | thread_num %d | pose_size %d | max_iter %d | part_length %d | gap_num %d | last_win_size %d | "
           "left_gap_num %d | tail %d | left_size %d | left_h_size %d | j_upper %d | "
           "downsample_size %f | voxel_size %f | eigen_ratio %f | reject_ratio %f\n",
           layer_num, thread_num, pose_size, max_iter, part_length, gap_num, last_win_size,
           left_gap_num, tail, left_size, left_h_size, j_upper,
           downsample_size, voxel_size, eigen_ratio, reject_ratio);
  }
};

class HBA
{
public:
  int thread_num, total_layer_num;
  std::vector<LAYER> layers;
  std::string data_path;

  HBA(int total_layer_num_, std::string data_path_, int thread_num_) // 总layer数，数据路径，线程数
  {
    total_layer_num = total_layer_num_;
    thread_num = thread_num_;
    data_path = data_path_;

    layers.resize(total_layer_num);
    for (int i = 0; i < total_layer_num; i++)
    {
      layers[i].layer_num = i + 1;
      layers[i].thread_num = thread_num;
    }
    layers[0].data_path = data_path;
    layers[0].pose_vec = mypcl::read_pose(data_path + "pose.json");
    layers[0].init_parameter();
    layers[0].init_storage(total_layer_num);

    for (int i = 1; i < total_layer_num; i++)
    {
      int pose_size_ = (layers[i - 1].thread_num - 1) * layers[i - 1].part_length;
      pose_size_ += layers[i - 1].tail == 0 ? layers[i - 1].left_gap_num : (layers[i - 1].left_gap_num + 1); // 若上一层没有剩余位姿，则直接加上最后一个线程的任务量（任务量是窗口移动次数）
      layers[i].init_parameter(pose_size_);
      layers[i].init_storage(total_layer_num);
      layers[i].data_path = layers[i - 1].data_path + "process1/";
    }
    printf("HBA init done!\n");
  }

  void update_next_layer_state(int cur_layer_num)
  {
    for (int i = 0; i < layers[cur_layer_num].thread_num; i++) // 遍历当前层的所有线程
      if (i < layers[cur_layer_num].thread_num - 1)            // 若不是最后一个线程
        for (int j = 0; j < layers[cur_layer_num].part_length; j++)
        {
          int index = (i * layers[cur_layer_num].part_length + j) * GAP; // 计算当前图层的某一个位姿的索引
          // 将当前图层的位姿的姿态赋值给下一图层的位姿索引，这里的位姿索引是按照线程分配的任务量来的，不乘GAP就相当于一次降采样？每一层处理的位姿数量是上一层的GAP倍？
          layers[cur_layer_num + 1].pose_vec[i * layers[cur_layer_num].part_length + j] = layers[cur_layer_num].pose_vec[index];
        }
      else // 最后一个线程
        for (int j = 0; j < layers[cur_layer_num].j_upper; j++)
        {
          int index = (i * layers[cur_layer_num].part_length + j) * GAP;
          layers[cur_layer_num + 1].pose_vec[i * layers[cur_layer_num].part_length + j] = layers[cur_layer_num].pose_vec[index];
        }
  }
  // 位姿图优化
  void pose_graph_optimization()
  {
    // 初始化位姿和Hessian矩阵
    std::vector<mypcl::pose> upper_pose, init_pose;
    upper_pose = layers[total_layer_num - 1].pose_vec; // 顶层位姿
    init_pose = layers[0].pose_vec;                    // 底层位姿
    std::vector<VEC(6)> upper_cov, init_cov;
    upper_cov = layers[total_layer_num - 1].hessians; // 顶层Hessian矩阵
    init_cov = layers[0].hessians;                    // 底层Hessian矩阵

    // 初始化gtsam
    int cnt = 0;
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Vector Vector6(6);
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8;
    gtsam::noiseModel::Diagonal::shared_ptr priorModel = gtsam::noiseModel::Diagonal::Variances(Vector6);                                                    // 先验噪声模型
    initial.insert(0, gtsam::Pose3(gtsam::Rot3(init_pose[0].q.toRotationMatrix()), gtsam::Point3(init_pose[0].t)));                                          // 插入底层位姿
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(gtsam::Rot3(init_pose[0].q.toRotationMatrix()), gtsam::Point3(init_pose[0].t)), priorModel)); // 添加先验因子，包括位姿和噪声模型

    // 遍历初始层的位姿，添加因子到因子图中：因子图是一个非线性因子图，包含了所有的因子，可以用于求解
    for (uint i = 0; i < init_pose.size(); i++)
    {
      if (i > 0)
        initial.insert(i, gtsam::Pose3(gtsam::Rot3(init_pose[i].q.toRotationMatrix()), gtsam::Point3(init_pose[i].t)));

      if (i % GAP == 0 && cnt < init_cov.size()) // 滑动窗口添加因子
        for (int j = 0; j < WIN_SIZE - 1; j++)   // 遍历窗口内的位姿
          for (int k = j + 1; k < WIN_SIZE; k++) // 遍历窗口内的位姿
          {
            if (i + j + 1 >= init_pose.size() || i + k >= init_pose.size())
              break; // 如果超出了位姿数量，跳出循环

            cnt++;
            if (init_cov[cnt - 1].norm() < 1e-20)
              continue; // 如果Hessian矩阵的范数小于1e-20，跳过

            Eigen::Vector3d t_ab = init_pose[i + j].t;
            Eigen::Matrix3d R_ab = init_pose[i + j].q.toRotationMatrix();
            t_ab = R_ab.transpose() * (init_pose[i + k].t - t_ab);           // 计算两个位姿之间的相对位移
            R_ab = R_ab.transpose() * init_pose[i + k].q.toRotationMatrix(); // 计算两个位姿之间的相对旋转
            gtsam::Rot3 R_sam(R_ab);
            gtsam::Point3 t_sam(t_ab);

            Vector6 << fabs(1.0 / init_cov[cnt - 1](0)), fabs(1.0 / init_cov[cnt - 1](1)), fabs(1.0 / init_cov[cnt - 1](2)),
                fabs(1.0 / init_cov[cnt - 1](3)), fabs(1.0 / init_cov[cnt - 1](4)), fabs(1.0 / init_cov[cnt - 1](5));
            gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6); // 噪声模型
            gtsam::NonlinearFactor::shared_ptr factor(new gtsam::BetweenFactor<gtsam::Pose3>(i + j, i + k, gtsam::Pose3(R_sam, t_sam),
                                                                                             odometryNoise)); // 添加因子
            graph.push_back(factor);
          }
    }

    // 遍历最高层的位姿，添加因子到因子图中：
    int pose_size = upper_pose.size();
    cnt = 0;
    for (int i = 0; i < pose_size - 1; i++)
      for (int j = i + 1; j < pose_size; j++)
      {
        cnt++;
        if (upper_cov[cnt - 1].norm() < 1e-20)
          continue;

        Eigen::Vector3d t_ab = upper_pose[i].t;
        Eigen::Matrix3d R_ab = upper_pose[i].q.toRotationMatrix();
        t_ab = R_ab.transpose() * (upper_pose[j].t - t_ab);
        R_ab = R_ab.transpose() * upper_pose[j].q.toRotationMatrix();
        gtsam::Rot3 R_sam(R_ab);
        gtsam::Point3 t_sam(t_ab);

        Vector6 << fabs(1.0 / upper_cov[cnt - 1](0)), fabs(1.0 / upper_cov[cnt - 1](1)), fabs(1.0 / upper_cov[cnt - 1](2)),
            fabs(1.0 / upper_cov[cnt - 1](3)), fabs(1.0 / upper_cov[cnt - 1](4)), fabs(1.0 / upper_cov[cnt - 1](5));
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
        gtsam::NonlinearFactor::shared_ptr factor(new gtsam::BetweenFactor<gtsam::Pose3>(i * pow(GAP, total_layer_num - 1),
                                                                                         j * pow(GAP, total_layer_num - 1), gtsam::Pose3(R_sam, t_sam), odometryNoise));
        graph.push_back(factor);
      }

    // 使用gtsam进行优化
    gtsam::ISAM2Params parameters;
    // 重新线性化阈值,由于非线性函数的特性，随着优化的进行，误差函数可能会发生较大的变化，导致线性化的误差逐渐增大。为了避免这种误差积累影响优化过程，通常会在优化的过程中引入 重新线性化 的机制
    parameters.relinearizeThreshold = 0.01;
    // 重新线性化跳过,设置=1，确保每次迭代都会重新线性化
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2 isam(parameters); // 创建ISAM2对象
    isam.update(graph, initial);
    isam.update();

    gtsam::Values results = isam.calculateEstimate(); // 计算估计值

    cout << "vertex size " << results.size() << endl;

    for (uint i = 0; i < results.size(); i++)
    {
      gtsam::Pose3 pose = results.at(i).cast<gtsam::Pose3>();
      assign_qt(init_pose[i].q, init_pose[i].t, Eigen::Quaterniond(pose.rotation().matrix()), pose.translation());
    }
    //mypcl::write_pose(init_pose, data_path); // 写入优化后的位姿
    mypcl::writeEVOPose(lidar_time, init_pose, data_path);
    printf("pgo complete\n");
  }
};

#endif