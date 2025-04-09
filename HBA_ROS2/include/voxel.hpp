#ifndef VOXEL_HPP
#define VOXEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cstring>
#include <thread>
#include <fstream>
#include <iomanip>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <Eigen/SparseCholesky>
#include <visualization_msgs/msg/marker.h>
#include <visualization_msgs/msg/marker_array.h>
#include <math.h>
#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "tools.hpp"
#include "layer.hpp"

#define AVG_THR
// #define ENABLE_RVIZ
// #define ENABLE_FILTER
using namespace std;

const double one_three = (1.0 / 3.0);

int layer_limit = 2;
int MIN_PT = 15;
int thd_num = 16;

class VOX_FACTOR
{
public:
  Eigen::Matrix3d P;
  Eigen::Vector3d v;
  int N;

  VOX_FACTOR()
  {
    P.setZero();
    v.setZero();
    N = 0;
  }

  void clear()
  {
    P.setZero();
    v.setZero();
    N = 0;
  }

  void push(const Eigen::Vector3d &vec)
  {
    N++;
    P += vec * vec.transpose();
    v += vec;
  }

  Eigen::Matrix3d cov()
  {
    Eigen::Vector3d center = v / N;
    return P/N - center*center.transpose();
  }

  VOX_FACTOR & operator+=(const VOX_FACTOR& sigv)
  {
    this->P += sigv.P;
    this->v += sigv.v;
    this->N += sigv.N;

    return *this;
  }

  void transform(const VOX_FACTOR &sigv, const IMUST &stat)
  {
    N = sigv.N;
    v = stat.R*sigv.v + N*stat.p;
    Eigen::Matrix3d rp = stat.R * sigv.v * stat.p.transpose();
    P = stat.R*sigv.P*stat.R.transpose() + rp + rp.transpose() + N*stat.p*stat.p.transpose();
  }
};

const double threshold = 0.1;
bool esti_plane(Eigen::Vector4d &pca_result, const pcl::PointCloud<PointType> &point)
{
  Eigen::Matrix<double, NMATCH, 3> A;
  Eigen::Matrix<double, NMATCH, 1> b;
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < NMATCH; j++) 
  {
    A(j, 0) = point[j].x;
    A(j, 1) = point[j].y;
    A(j, 2) = point[j].z;
  }

  Eigen::Vector3d normvec = A.colPivHouseholderQr().solve(b);

  for (int j = 0; j < NMATCH; j++) 
  {
    if (fabs(normvec.dot(A.row(j)) + 1.0) > threshold) 
      return false;
  }

  double n = normvec.norm();
  pca_result(0) = normvec(0) / n;
  pca_result(1) = normvec(1) / n;
  pca_result(2) = normvec(2) / n;
  pca_result(3) = 1.0 / n;
  return true;
}

class VOX_HESS
{
public:
    // sig_orig是某个体素内点云特征的容器，plvec_voxel就是用于存储这些容器，vec_orig是某个体素内点云位置的容器
    vector<const vector<VOX_FACTOR>*> plvec_voxels; // 体素因子向量集，元素包括（P, v, N）, 方法有clear, push, cov, operator+=, transform
    vector<PLV(3)> origin_points;                    // 点集的集合(3层)，每一次origin_points[i]都是一个vector<Eigen::Matrix<double, 3, 1>>
    int win_size;                                    // 窗口大小

    VOX_HESS(int _win_size = WIN_SIZE) : win_size(_win_size) { origin_points.resize(win_size); }

    ~VOX_HESS()
    {
        vector<const vector<VOX_FACTOR>*>().swap(plvec_voxels);
    }

    // 将vec_orig中的点添加到origin_points_的末尾
    void get_center(const PLV(3) & vec_orig, PLV(3) & origin_points_)
    {
        size_t pc_size = vec_orig.size();
        for (size_t i = 0; i < pc_size; i++)
        {
            origin_points_.emplace_back(vec_orig[i]);
        }
        return;
    }

    // sig_orig是某个体素内点云特征的容器，plvec_voxel就是用于存储这些容器
    void push_voxel(const vector<VOX_FACTOR> *sig_orig, const vector<PLV(3)> *vec_orig)
    {
        int process_size = 0;
        // 计算有效体素的数量
        for (int i = 0; i < win_size; i++)
            if ((*sig_orig)[i].N != 0)
                process_size++;

#ifdef ENABLE_FILTER
        // 如果有效体素的数量小于1，则直接返回
        if (process_size < 1)
            return;

        // 计算每个非零体素的中心
        for (int i = 0; i < win_size; i++)
            if ((*sig_orig)[i].N != 0)
                get_center((*vec_orig)[i], origin_points[i]);
#endif

        // 如果有效体素的数量小于2，则直接返回
        if (process_size < 2)
            return;

        // 将输入的原始信号向量添加到 plvec_voxels 容器中
        plvec_voxels.push_back(sig_orig);
    }

    /// @brief 计算体素内窗口内的Hessian矩阵，雅各布矩阵和残差，两个矩阵见formula1.png和formula2.png推导见Reference，这个残差是每个窗口的最小的特征值的和
    /// @param xs 状态量: R,P,V,bg,ba,g,t
    /// @param head
    /// @param end
    /// @param Hess
    /// @param JacT
    /// @param residual
    void acc_evaluate2(const vector<IMUST> &xs, int head, int end, Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT, double &residual)
    {
        // 初始化
        Hess.setZero();
        JacT.setZero();
        residual = 0;
        vector<VOX_FACTOR> sig_tran(win_size);
        const int kk = 0;

        PLV(3)
        viRiTuk(win_size);
        PLM(3)
        viRiTukukT(win_size);

        vector<Eigen::Matrix<double, 3, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 6>>> Auk(win_size);
        Eigen::Matrix3d umumT;

        for (int a = head; a < end; a++)
        {
            // 对于体素 a，获取其原始数据 sig_orig
            const vector<VOX_FACTOR> &sig_orig = *plvec_voxels[a];

            VOX_FACTOR sig;

            // 对于每个窗口 i，将 sig_orig[i] 通过 xs[i] 转换到 xs的坐标系下，包括协方差，质心和点数量
            for (int i = 0; i < win_size; i++)
            {
                if (sig_orig[i].N != 0)
                {
                    sig_tran[i].transform(sig_orig[i], xs[i]);
                    sig += sig_tran[i];
                }
            }
            // 质心坐标
            const Eigen::Vector3d &vBar = sig.v / sig.N;

            // 体素内点云的协方差矩阵,并进行特征值分解
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sig.P / sig.N - vBar * vBar.transpose());
            // 特征值，这个特征值是按照从小到大排列的
            const Eigen::Vector3d &lmbd = saes.eigenvalues();
            // 特征向量组成的矩阵
            const Eigen::Matrix3d &U = saes.eigenvectors();
            // 记录体素内点的数量
            int NN = sig.N;

            // 特征向量
            Eigen::Vector3d u[3] = {U.col(0), U.col(1), U.col(2)};

            // 计算雅各布矩阵
            const Eigen::Vector3d &uk = u[kk]; // kk=0, 这里就是uk = u(0)，也就是第一个特征向量
            Eigen::Matrix3d ukukT = uk * uk.transpose();
            umumT.setZero();
            for (int i = 0; i < 3; i++)
            {
                if (i != kk)
                {
                    umumT += 2.0 / (lmbd[kk] - lmbd[i]) * u[i] * u[i].transpose();
                }
            }

            // 计算hessian矩阵，这个hessian矩阵在formula1.png和formula2.png中，具体推导参考论文Reference的第8到11页
            for (int i = 0; i < win_size; i++)
            {
                if (sig_orig[i].N != 0)
                {
                    Eigen::Matrix3d Pi = sig_orig[i].P;
                    Eigen::Vector3d vi = sig_orig[i].v;
                    Eigen::Matrix3d Ri = xs[i].R;
                    double ni = sig_orig[i].N;

                    Eigen::Matrix3d vihat;
                    vihat << SKEW_SYM_MATRX(vi); // 反对称矩阵
                    Eigen::Vector3d RiTuk = Ri.transpose() * uk;
                    Eigen::Matrix3d RiTukhat;
                    RiTukhat << SKEW_SYM_MATRX(RiTuk);

                    Eigen::Vector3d PiRiTuk = Pi * RiTuk;
                    viRiTuk[i] = vihat * RiTuk;
                    viRiTukukT[i] = viRiTuk[i] * uk.transpose();

                    Eigen::Vector3d ti_v = xs[i].p - vBar;
                    double ukTti_v = uk.dot(ti_v);

                    Eigen::Matrix3d combo1 = hat(PiRiTuk) + vihat * ukTti_v;
                    Eigen::Vector3d combo2 = Ri * vi + ni * ti_v;
                    Auk[i].block<3, 3>(0, 0) = (Ri * Pi + ti_v * vi.transpose()) * RiTukhat - Ri * combo1;
                    Auk[i].block<3, 3>(0, 3) = combo2 * uk.transpose() + combo2.dot(uk) * I33;
                    Auk[i] /= NN;

                    const Eigen::Matrix<double, 6, 1> &jjt = Auk[i].transpose() * uk;
                    JacT.block<6, 1>(6 * i, 0) += jjt;

                    const Eigen::Matrix3d &HRt = 2.0 / NN * (1.0 - ni / NN) * viRiTukukT[i];
                    Eigen::Matrix<double, 6, 6> Hb = Auk[i].transpose() * umumT * Auk[i];
                    Hb.block<3, 3>(0, 0) += 2.0 / NN * (combo1 - RiTukhat * Pi) * RiTukhat - 2.0 / NN / NN * viRiTuk[i] * viRiTuk[i].transpose() - 0.5 * hat(jjt.block<3, 1>(0, 0));
                    Hb.block<3, 3>(0, 3) += HRt;
                    Hb.block<3, 3>(3, 0) += HRt.transpose();
                    Hb.block<3, 3>(3, 3) += 2.0 / NN * (ni - ni * ni / NN) * ukukT;

                    Hess.block<6, 6>(6 * i, 6 * i) += Hb;
                }
            }

            for (int i = 0; i < win_size - 1; i++)
            {
                if (sig_orig[i].N != 0)
                {
                    double ni = sig_orig[i].N;
                    for (int j = i + 1; j < win_size; j++)
                    {
                        if (sig_orig[j].N != 0)
                        {
                            double nj = sig_orig[j].N;
                            Eigen::Matrix<double, 6, 6> Hb = Auk[i].transpose() * umumT * Auk[j];
                            Hb.block<3, 3>(0, 0) += -2.0 / NN / NN * viRiTuk[i] * viRiTuk[j].transpose();
                            Hb.block<3, 3>(0, 3) += -2.0 * nj / NN / NN * viRiTukukT[i];
                            Hb.block<3, 3>(3, 0) += -2.0 * ni / NN / NN * viRiTukukT[j].transpose();
                            Hb.block<3, 3>(3, 3) += -2.0 * ni * nj / NN / NN * ukukT;

                            Hess.block<6, 6>(6 * i, 6 * j) += Hb;
                        }
                    }
                }
            }

            residual += lmbd[kk];
        }

        for (int i = 1; i < win_size; i++)
        {
            for (int j = 0; j < i; j++)
            {
                Hess.block<6, 6>(6 * i, 6 * j) = Hess.block<6, 6>(6 * j, 6 * i).transpose();
            }
        }
    }

    // 只计算残差，不计算雅各布矩阵和Hessian矩阵，这个计算的也是所有体素内每个窗口的最小特征值
    void evaluate_only_residual(const vector<IMUST> &xs, double &residual)
    {
        residual = 0;
        vector<VOX_FACTOR> sig_tran(win_size);
        int kk = 0; // The kk-th lambda value

        int gps_size = plvec_voxels.size();

        for (int a = 0; a < gps_size; a++)
        {
            const vector<VOX_FACTOR> &sig_orig = *plvec_voxels[a];
            VOX_FACTOR sig;

            for (int i = 0; i < win_size; i++)
            {
                sig_tran[i].transform(sig_orig[i], xs[i]);
                sig += sig_tran[i];
            }

            Eigen::Vector3d vBar = sig.v / sig.N;
            Eigen::Matrix3d cmt = sig.P / sig.N - vBar * vBar.transpose();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cmt);
            Eigen::Vector3d lmbd = saes.eigenvalues();

            residual += lmbd[kk];
        }
    }

    // 和上面的evaluate_only_residual类似，只是这个函数返回的是一个vector<double>，这个vector<double>是所有体素内每个窗口的最小特征值
    // 主要用于去除离群点
    std::vector<double> evaluate_residual(const vector<IMUST> &xs)
    {
        /* for outlier removal usage */
        std::vector<double> residuals;
        vector<VOX_FACTOR> sig_tran(win_size);
        int kk = 0; // The kk-th lambda value
        int gps_size = plvec_voxels.size();

        for (int a = 0; a < gps_size; a++)
        {
            const vector<VOX_FACTOR> &sig_orig = *plvec_voxels[a];
            VOX_FACTOR sig;

            for (int i = 0; i < win_size; i++)
            {
                sig_tran[i].transform(sig_orig[i], xs[i]);
                sig += sig_tran[i];
            }

            Eigen::Vector3d vBar = sig.v / sig.N;
            Eigen::Matrix3d cmt = sig.P / sig.N - vBar * vBar.transpose();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cmt);
            Eigen::Vector3d lmbd = saes.eigenvalues();

            residuals.push_back(lmbd[kk]);
        }

        return residuals;
    }

    // 主要用于计算需要去除离群点的数量
    void remove_residual(const vector<IMUST> &xs, double threshold, double reject_num)
    {
        vector<VOX_FACTOR> sig_tran(win_size);
        int kk = 0; // The kk-th lambda value
        int rej_cnt = 0;
        size_t i = 0;
        for (; i < plvec_voxels.size();)
        {
            const vector<VOX_FACTOR> &sig_orig = *plvec_voxels[i];
            VOX_FACTOR sig;

            for (int j = 0; j < win_size; j++)
            {
                sig_tran[j].transform(sig_orig[j], xs[j]);
                sig += sig_tran[j];
            }

            Eigen::Vector3d vBar = sig.v / sig.N;
            Eigen::Matrix3d cmt = sig.P / sig.N - vBar * vBar.transpose();
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cmt);
            Eigen::Vector3d lmbd = saes.eigenvalues();

            if (lmbd[kk] >= threshold)
            {
                plvec_voxels.erase(plvec_voxels.begin() + i);
                rej_cnt++;
                continue;
            }
            i++;
            if (rej_cnt == reject_num)
            {
                break;
            }
        }
    }
};

// 类存储体素的位置信息
class VOXEL_LOC
{
public:
    int64_t x, y, z;

    VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

    bool operator==(const VOXEL_LOC &other) const
    {
        return (x == other.x && y == other.y && z == other.z);
    }
};

namespace std
{
    template <>
    struct hash<VOXEL_LOC>
    {
        size_t operator()(const VOXEL_LOC &s) const
        {
            using std::hash;
            using std::size_t;
            // return (((hash<int64_t>()(s.z)*HASH_P)%MAX_N + hash<int64_t>()(s.y))*HASH_P)%MAX_N + hash<int64_t>()(s.x);
            long long index_x, index_y, index_z;
            double cub_len = 0.125;
            index_x = int(round(floor((s.x) / cub_len + SMALL_EPS)));
            index_y = int(round(floor((s.y) / cub_len + SMALL_EPS)));
            index_z = int(round(floor((s.z) / cub_len + SMALL_EPS)));
            return (((((index_z * HASH_P) % MAX_N + index_y) * HASH_P) % MAX_N) + index_x) % MAX_N;
        }
    };
}

int BINGO_CNT = 0;
enum OCTO_STATE
{
    UNKNOWN,
    MID_NODE,
    PLANE
}; // 八叉树的状态, 未知，中间节点，平面

// 八叉树节点类，目前看来，这个八叉树主要是存储体素的特征，当判断完体素的特征后，就将这个节点里存储的数据清空，然后将这个节点的八个子节点存储数据
class OCTO_TREE_NODE
{
public:
    OCTO_STATE octo_state; // 八叉树的状态
    int layer;             // 八叉树的层数
    int win_size;          // 窗口大小
    vector<PLV(3)> vec_orig, vec_tran;
    vector<VOX_FACTOR> sig_orig, sig_tran;

    OCTO_TREE_NODE *leaves[8]; // 八叉树的子节点
    float voxel_center[3];     // 体素中心坐标
    float quater_length;       // 体素的四分之一边长
    float eigen_thr;

    Eigen::Vector3d center, direct, value_vector; // 体素的质心、特征向量和特征值
    double eigen_ratio;

// 选择性发布残差以及方向的值
#ifdef ENABLE_RVIZ
    ros::NodeHandle nh;
    ros::Publisher pub_residual = nh.advertise<sensor_msgs::PointCloud2>("/residual", 1000);
    ros::Publisher pub_direct = nh.advertise<visualization_msgs::MarkerArray>("/direct", 1000);
#endif

    OCTO_TREE_NODE(int _win_size = WIN_SIZE, float _eigen_thr = 1.0 / 10) : win_size(_win_size), eigen_thr(_eigen_thr)
    {
        octo_state = UNKNOWN;
        layer = 0;
        vec_orig.resize(win_size);
        vec_tran.resize(win_size);
        sig_orig.resize(win_size);
        sig_tran.resize(win_size);

        for (int i = 0; i < 8; i++)
        {
            leaves[i] = nullptr;
        }
    }

    virtual ~OCTO_TREE_NODE()
    {
        for (int i = 0; i < 8; i++)
        {
            if (leaves[i] != nullptr)
            {
                delete leaves[i];
            }
        }
    }

    /// @brief  判断体素是否是平面特征,0表示不是平面特征，1表示是平面特征
    bool judge_eigen()
    {
        VOX_FACTOR covMat;
        for (int i = 0; i < win_size; i++)
        {
            if (sig_tran[i].N > 0)
            {
                covMat += sig_tran[i];
            }
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat.cov());
        value_vector = saes.eigenvalues();   // 特征值
        center = covMat.v / covMat.N;        // 质心
        direct = saes.eigenvectors().col(0); // 特征向量

        eigen_ratio = saes.eigenvalues()[0] / saes.eigenvalues()[2]; // 扁率，越小约趋近于平面
        if (eigen_ratio > eigen_thr)                                 // 直接判断不是平面
        {
            return 0;
        }

        double eva0 = saes.eigenvalues()[0];
        double sqrt_eva0 = sqrt(eva0);
        Eigen::Vector3d center_turb = center + 5 * sqrt_eva0 * direct; // 扰动后的体素中心坐标

        vector<VOX_FACTOR> covMats(8); // 8个子体素的体素特征容器
        for (int i = 0; i < win_size; i++)
        {
            for (Eigen::Vector3d ap : vec_tran[i])
            {
                int xyz[3] = {0, 0, 0};
                for (int k = 0; k < 3; k++)
                {
                    if (ap(k) > center_turb[k])
                    {
                        xyz[k] = 1;
                    }
                }

                Eigen::Vector3d pvec(ap(0), ap(1), ap(2));

                int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2]; // 子体素的索引
                covMats[leafnum].push(pvec);
            }
        }

        // 根据子体素特征值判断父体素的特征
        double ratios[2] = {1.0 / (3.0 * 3.0), 2.0 * 2.0}; // 特征值比值的范围
        int num_all = 0, num_qua = 0;
        for (int i = 0; i < 8; i++)
        {
            if (covMats[i].N > 10)
            {
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMats[i].cov()); // 计算8个子体素内的协方差矩阵的特征值
                double child_eva0 = (saes.eigenvalues()[0]);                           // 子体素内的最小特征值
                if (child_eva0 > ratios[0] * eva0 && child_eva0 < ratios[1] * eva0)    // 如果子体素内的最小特征值在特定范围内
                {
                    num_qua++;
                }
                num_all++;
            }
        }

        double prop = 1.0 * num_qua / num_all; // 比例

        if (prop < 0.5)
        {
            return 0;
        }
        return 1;
    }

    // 将第ci个体素内的点云数据存储到八叉树的子节点中
    void cut_func(int ci)
    {
        PLV(3) &pvec_orig = vec_orig[ci];
        PLV(3) &pvec_tran = vec_tran[ci];

        // 将8个子体素存进8叉树的子树中, 这里遍历体素ci内的所有点云
        uint a_size = pvec_tran.size();
        for (uint j = 0; j < a_size; j++)
        {
            int xyz[3] = {0, 0, 0};
            for (uint k = 0; k < 3; k++)
            {
                if (pvec_tran[j][k] > voxel_center[k])
                {
                    xyz[k] = 1;
                }
            }
            int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2]; // 存子体素的中心点三维坐标
            if (leaves[leafnum] == nullptr)
            {
                leaves[leafnum] = new OCTO_TREE_NODE(win_size, eigen_thr);
                leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
                leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
                leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
                leaves[leafnum]->quater_length = quater_length / 2.0;
                leaves[leafnum]->layer = layer + 1;
            }
            leaves[leafnum]->vec_orig[ci].push_back(pvec_orig[j]);
            leaves[leafnum]->vec_tran[ci].push_back(pvec_tran[j]);

            leaves[leafnum]->sig_orig[ci].push(pvec_orig[j]);
            leaves[leafnum]->sig_tran[ci].push(pvec_tran[j]);
        }

        PLV(3)().swap(pvec_orig);
        PLV(3)().swap(pvec_tran);
    }

    // 主要是确定体素的在八叉树中的特征
    void recut()
    {
        if (octo_state == UNKNOWN)
        {
            int point_size = 0;
            for (int i = 0; i < win_size; i++)
                point_size += sig_orig[i].N;

            if (point_size < MIN_PT)
            {
                octo_state = MID_NODE;
                vector<PLV(3)>().swap(vec_orig); // 释放内存
                vector<PLV(3)>().swap(vec_tran);
                vector<VOX_FACTOR>().swap(sig_orig);
                vector<VOX_FACTOR>().swap(sig_tran);
                return;
            }

            if (judge_eigen()) // 判断是否是平面特征
            {
                octo_state = PLANE;
#ifndef ENABLE_FILTER
#ifndef ENABLE_RVIZ
                vector<PLV(3)>().swap(vec_orig);
                vector<PLV(3)>().swap(vec_tran);
#endif
#endif
                return;
            }
            else
            {
                if (layer == layer_limit)
                {
                    octo_state = MID_NODE;
                    vector<PLV(3)>().swap(vec_orig);
                    vector<PLV(3)>().swap(vec_tran);
                    vector<VOX_FACTOR>().swap(sig_orig);
                    vector<VOX_FACTOR>().swap(sig_tran);
                    return;
                }
                vector<VOX_FACTOR>().swap(sig_orig);
                vector<VOX_FACTOR>().swap(sig_tran);
                for (int i = 0; i < win_size; i++)
                    cut_func(i);
            }
        }

        for (int i = 0; i < 8; i++)
            if (leaves[i] != nullptr)
                leaves[i]->recut();
    }

    // 向体素容器中添加体素的特征、点云数据(ifdef ENABLE_FILTER)
    void tras_opt(VOX_HESS &vox_opt)
    {
        // 如果当前节点(也就是这个体素)的状态是 PLANE
        if (octo_state == PLANE)
        {
            vox_opt.push_voxel(&sig_orig, &vec_orig); // 向体素优化器中添加体素内的点云特征和点云数据
        }
        else
        {
            for (int i = 0; i < 8; i++)
            {
                if (leaves[i] != nullptr)
                {
                    leaves[i]->tras_opt(vox_opt);
                }
            }
        }
    }

    void tras_display(int layer = 0)
    {
        float ref = 255.0 * rand() / (RAND_MAX + 1.0f);
        pcl::PointXYZINormal ap;
        ap.intensity = ref;

        if (octo_state == PLANE)
        {
            // std::vector<unsigned int> colors;
            // colors.push_back(static_cast<unsigned int>(rand() % 256));
            // colors.push_back(static_cast<unsigned int>(rand() % 256));
            // colors.push_back(static_cast<unsigned int>(rand() % 256));
            pcl::PointCloud<pcl::PointXYZINormal> color_cloud;

            for (int i = 0; i < win_size; i++)
            {
                for (size_t j = 0; j < vec_tran[i].size(); j++)
                {
                    Eigen::Vector3d &pvec = vec_tran[i][j];
                    ap.x = pvec.x();
                    ap.y = pvec.y();
                    ap.z = pvec.z();
                    // ap.b = colors[0];
                    // ap.g = colors[1];
                    // ap.r = colors[2];
                    ap.normal_x = sqrt(value_vector[1] / value_vector[0]);
                    ap.normal_y = sqrt(value_vector[2] / value_vector[0]);
                    ap.normal_z = sqrt(value_vector[0]);
                    // ap.curvature = total;
                    color_cloud.push_back(ap);
                }
            }

#ifdef ENABLE_RVIZ
            sensor_msgs::msg::PointCloud2 dbg_msg;
            pcl::toROSMsg(color_cloud, dbg_msg);
            dbg_msg.header.frame_id = "camera_init";
            pub_residual.publish(dbg_msg);

            visualization_msgs::Marker marker;
            visualization_msgs::MarkerArray marker_array;
            marker.header.frame_id = "camera_init";
            marker.header.stamp = ros::Time::now();
            marker.ns = "basic_shapes";
            marker.id = BINGO_CNT;
            BINGO_CNT++;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.color.a = 1;
            marker.color.r = layer == 0 ? 1 : 0;
            marker.color.g = layer == 1 ? 1 : 0;
            marker.color.b = layer == 2 ? 1 : 0;
            marker.scale.x = 0.01;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.lifetime = ros::Duration();
            geometry_msgs::Point apoint;
            apoint.x = center(0);
            apoint.y = center(1);
            apoint.z = center(2);
            marker.points.push_back(apoint);
            apoint.x += 0.2 * direct(0);
            apoint.y += 0.2 * direct(1);
            apoint.z += 0.2 * direct(2);
            marker.points.push_back(apoint);
            marker_array.markers.push_back(marker);
            pub_direct.publish(marker_array);
#endif
        }
        else
        {
            if (layer == layer_limit)
                return;
            layer++;
            for (int i = 0; i < 8; i++)
                if (leaves[i] != nullptr)
                    leaves[i]->tras_display(layer);
        }
    }
};

class OCTO_TREE_ROOT : public OCTO_TREE_NODE
{
public:
    OCTO_TREE_ROOT(int _winsize, float _eigen_thr) : OCTO_TREE_NODE(_winsize, _eigen_thr) {}
};

class VOX_OPTIMIZER
{
public:
    int win_size, jac_leng, imu_leng;
    VOX_OPTIMIZER(int _win_size = WIN_SIZE) : win_size(_win_size)
    {
        jac_leng = DVEL * win_size;
        imu_leng = DIM * win_size;
    }

    // 分线程计算，计算每个线程的残差, 返回所有线程的总残差（也就是所有体素的总残差）的平均值（ifdef avg_thr）或总残差
    double divide_thread(vector<IMUST> &x_stats, VOX_HESS &voxhess, vector<IMUST> &x_ab, Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT)
    {
        double residual = 0;
        Hess.setZero();
        JacT.setZero();
        PLM(-1)
        hessians(thd_num); // 动态大小的thd_num个hessian矩阵
        PLV(-1)
        jacobins(thd_num);

        for (int i = 0; i < thd_num; i++)
        {
            hessians[i].resize(jac_leng, jac_leng);
            jacobins[i].resize(jac_leng);
        }

        int tthd_num = thd_num;
        vector<double> resis(tthd_num, 0); // 存储每个线程的残差
        int g_size = voxhess.plvec_voxels.size();
        if (g_size < tthd_num) // 如果体素的数量小于线程数
        {
            tthd_num = 1;
        }

        vector<thread *> mthreads(tthd_num);
        double part = 1.0 * g_size / tthd_num;
        for (int i = 0; i < tthd_num; i++)
        {
            // 开启线程计算每个线程的残差，雅各布矩阵和Hessian矩阵, 然后所有体素的残差，雅各布矩阵和Hessian矩阵累加得到总的Hessian矩阵，雅各布矩阵和残差
            mthreads[i] = new thread(&VOX_HESS::acc_evaluate2, &voxhess, x_stats, part * i, part * (i + 1),
                                     ref(hessians[i]), ref(jacobins[i]), ref(resis[i]));
        }

        for (int i = 0; i < tthd_num; i++)
        {
            mthreads[i]->join();
            Hess += hessians[i];
            JacT += jacobins[i];
            residual += resis[i]; // 所有线程的总残差
            delete mthreads[i];
        }
#ifdef AVG_THR
        return residual / g_size;
#else
        return residual;
#endif
    }

    double only_residual(vector<IMUST> &x_stats, VOX_HESS &voxhess, vector<IMUST> &x_ab, bool is_avg = false)
    {
        double residual2 = 0;
        voxhess.evaluate_only_residual(x_stats, residual2);
        if (is_avg)
            return residual2 / voxhess.plvec_voxels.size();
        return residual2;
    }

    void remove_outlier(vector<IMUST> &x_stats, VOX_HESS &voxhess, double ratio)
    {
        // 计算残差
        std::vector<double> residuals = voxhess.evaluate_residual(x_stats);

        // 对残差进行升序排序
        std::sort(residuals.begin(), residuals.end());

        // 计算阈值, 选取前ratio的体素的残差的最大值作为阈值
        double threshold = residuals[std::floor((1 - ratio) * voxhess.plvec_voxels.size()) - 1];

        // 计算所需要移除的离群点的数量, 选取ratio的体素的数量作为需要移除的离群点的 最大 数量
        int reject_num = std::floor(ratio * voxhess.plvec_voxels.size());

        // 移除离群点
        voxhess.remove_residual(x_stats, threshold, reject_num);
    }

    void damping_iter(vector<IMUST> &x_stats, VOX_HESS &voxhess, double &residual, PLV(6) & hess_vec, size_t &mem_cost)
    {
        double u = 0.01, v = 2;
        Eigen::MatrixXd D(jac_leng, jac_leng), Hess(jac_leng, jac_leng),
            HessuD(jac_leng, jac_leng);
        Eigen::VectorXd JacT(jac_leng), dxi(jac_leng), new_dxi(jac_leng);

        D.setIdentity();
        double residual1, residual2, q;

        bool is_calc_hess = true;

        vector<IMUST> x_stats_temp;
        vector<IMUST> x_ab(win_size);
        x_ab[0] = x_stats[0];

        // 计算a,b两点之间的相对位姿
        for (int i = 1; i < win_size; i++)
        {
            x_ab[i].p = x_stats[i - 1].R.transpose() * (x_stats[i].p - x_stats[i - 1].p);
            x_ab[i].R = x_stats[i - 1].R.transpose() * x_stats[i].R;
        }

        double hesstime = 0;
        double solve_time = 0;
        size_t max_mem = 0;
        double loop_num = 0;
        for (int i = 0; i < 10; i++)
        {
            if (is_calc_hess)
            {
                double tm = rclcpp::Clock().now().seconds();
                residual1 = divide_thread(x_stats, voxhess, x_ab, Hess, JacT); // 所有线程残差（也就是所有体素）的平均值
                hesstime += rclcpp::Clock().now().seconds() - tm;
            }

            double tm = rclcpp::Clock().now().seconds();
            D.diagonal() = Hess.diagonal();
            HessuD = Hess + u * D;
            double t1 = rclcpp::Clock().now().seconds();
            Eigen::SparseMatrix<double> A1_sparse(jac_leng, jac_leng);
            std::vector<Eigen::Triplet<double>> tripletlist;
            for (int a = 0; a < jac_leng; a++)
            {
                for (int b = 0; b < jac_leng; b++)
                {
                    if (HessuD(a, b) != 0)
                    {
                        tripletlist.push_back(Eigen::Triplet<double>(a, b, HessuD(a, b)));
                    }
                }
            }
            A1_sparse.setFromTriplets(tripletlist.begin(), tripletlist.end()); // 根据tripletlist中的元素构建稀疏矩阵
            A1_sparse.makeCompressed();                                        // 将稀疏矩阵转化为压缩存储格式

            // compute() 方法对稀疏矩阵 A1_sparse 进行稀疏 LDLT 分解，将矩阵分解为 A= LDLT 的形式
            Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> Solver_sparse;
            Solver_sparse.compute(A1_sparse);

            // 返回当前内存使用量
            size_t temp_mem = check_mem();
            if (temp_mem > max_mem)
            {
                max_mem = temp_mem;
            }

            // 求解线性方程组 A1_sparse * x = -JacT
            dxi = Solver_sparse.solve(-JacT);
            temp_mem = check_mem();
            if (temp_mem > max_mem)
                max_mem = temp_mem;
            solve_time += rclcpp::Clock().now().seconds() - tm;
            // new_dxi = Solver_sparse.solve(-JacT);
            // printf("new solve time cost %f\n",rclcpp::Clock().now() - t1);
            // relative_err = ((Hess + u*D)*dxi + JacT).norm()/JacT.norm();
            // absolute_err = ((Hess + u*D)*dxi + JacT).norm();
            // std::cout<<"relative error "<<relative_err<<std::endl;
            // std::cout<<"absolute error "<<absolute_err<<std::endl;
            // std::cout<<"delta x\n"<<(new_dxi-dxi).transpose()/dxi.norm()<<std::endl;

            x_stats_temp = x_stats;
            for (int j = 0; j < win_size; j++)
            {
                x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(DVEL * j, 0));
                x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(DVEL * j + 3, 0);
            }

            double q1 = 0.5 * dxi.dot(u * D * dxi - JacT); // 0.5 * dxi与(u * D * dxi - JacT)的点积
#ifdef AVG_THR
            residual2 = only_residual(x_stats_temp, voxhess, x_ab, true); // 这个就是计算
            q1 /= voxhess.plvec_voxels.size();
#else
            residual2 = only_residual(x_stats_temp, voxhess, x_ab);
#endif

            residual = residual2;
            q = (residual1 - residual2);

            loop_num = i + 1;

            if (q > 0)
            {
                x_stats = x_stats_temp;
                q = q / q1;
                v = 2;
                q = 1 - pow(2 * q - 1, 3);
                u *= (q < one_three ? one_three : q);
                is_calc_hess = true;
            }
            else
            {
                u = u * v;
                v = 2 * v;
                is_calc_hess = false;
            }
#ifdef AVG_THR
            if ((fabs(residual1 - residual2) / residual1) < 0.05 || i == 9)
            {
                if (mem_cost < max_mem)
                    mem_cost = max_mem;
                for (int j = 0; j < win_size - 1; j++)
                    for (int k = j + 1; k < win_size; k++)
                        hess_vec.push_back(Hess.block<DVEL, DVEL>(DVEL * j, DVEL * k).diagonal().segment<DVEL>(0));
                break;
            }
#else
            if (fabs(residual1 - residual2) < 1e-9)
                break;
#endif
        }
    }

    // 检查内存使用量
    size_t check_mem()
    {
        FILE *file = fopen("/proc/self/status", "r");
        int result = -1;
        char line[128];

        while (fgets(line, 128, file) != nullptr)
        {
            if (strncmp(line, "VmRSS:", 6) == 0)
            {
                int len = strlen(line);

                const char *p = line;
                for (; std::isdigit(*p) == false; ++p)
                {
                }

                line[len - 3] = 0;
                result = atoi(p);

                break;
            }
        }
        fclose(file);

        return result;
    }
};

void downsample_voxel(pcl::PointCloud<PointType> &pc, double voxel_size)
{
    if (voxel_size < 0.01)
        return;

    std::unordered_map<VOXEL_LOC, M_POINT> feature_map;
    size_t pt_size = pc.size();
    // 遍历所有的点云，将点的坐标除以体素大小，得到体素的位置。如果坐标小于 0，则减去 1.0
    for (size_t i = 0; i < pt_size; i++)
    {
        PointType &pt_trans = pc[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++)
        {
            loc_xyz[j] = pt_trans.data[j] / voxel_size;
            if (loc_xyz[j] < 0)
                loc_xyz[j] -= 1.0;
        }
        // 在 feature_map 中查找该体素位置。如果找到，则累加该体素内的点坐标，并增加计数。,如果未找到，则创建一个新的 M_POINT 对象，并将其添加到 feature_map 中。
        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = feature_map.find(position);
        if (iter != feature_map.end())
        {
            iter->second.xyz[0] += pt_trans.x;
            iter->second.xyz[1] += pt_trans.y;
            iter->second.xyz[2] += pt_trans.z;
            iter->second.intensity += pt_trans.intensity;
            iter->second.count++;
        }
        else
        {
            M_POINT anp;
            anp.xyz[0] = pt_trans.x;
            anp.xyz[1] = pt_trans.y;
            anp.xyz[2] = pt_trans.z;
            anp.intensity = pt_trans.intensity;
            anp.count = 1;
            feature_map[position] = anp;
        }
    }

    // 将点云数据的大小更新为 feature_map 的大小。
    pt_size = feature_map.size();
    pc.clear();
    pc.resize(pt_size);

    // 遍历 feature_map 中的每个体素，计算每个体素内点的平均值，并更新点云数据。
    size_t i = 0;
    for (auto iter = feature_map.begin(); iter != feature_map.end(); ++iter)
    {
        pc[i].x = iter->second.xyz[0] / iter->second.count;
        pc[i].y = iter->second.xyz[1] / iter->second.count;
        pc[i].z = iter->second.xyz[2] / iter->second.count;
        pc[i].intensity = iter->second.intensity;
        i++;
    }
}
#endif