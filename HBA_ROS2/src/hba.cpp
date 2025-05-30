#include "hba.hpp"

std::vector<GPS_Factor::gps_imu_pose3d> gps_pose_vec;


HBA::HBA(int total_layer_num_, std::string data_path_, int thread_num_, GPS_Factor &gps_factor_func)
{
    total_layer_num = total_layer_num_;
    data_path = data_path_;
    thread_num = thread_num_;

    layers.resize(total_layer_num);
    for (int i = 0; i < total_layer_num; i++)
    {
        layers[i].layer_num = i + 1; // 索引
        layers[i].thread_num = thread_num;
    }
    layers[0].data_path = data_path;
    std::vector<mypcl::pose> lio_pose_orig = mypcl::read_pose(data_path + "pose.json");
    std::vector<mypcl::pose> lio_pose_tran;
    //layers[0].pose_vec
    if(enable_gps_factor == true)
    {
        if(gps_imu_info == true)
        {
            gps_pose_vec = gps_factor_func.read_gps_imu_data(data_path + "gps_imu_data.json");
        }
        else
        {
            gps_pose_vec = gps_factor_func.read_gps_raw_info(data_path + "gps_raw_data.json", data_path);
        }
        std::cout << "gps pose size = " << gps_pose_vec.size() << std::endl;
        std::cout << "lio pose size = " << lio_pose_orig.size() << std::endl;
        index_interpolate.reserve(gps_pose_vec.size());
        lio_pose_tran.reserve(gps_pose_vec.size() + lio_pose_orig.size());

        // 执行GPS点插值前请备份你的PCD文件
        interpolate_pose(lio_pose_orig, lio_pose_tran, gps_factor_func.gps_time, lidar_time);
        layers[0].pose_vec = lio_pose_tran;
    }
    else
    {
        layers[0].pose_vec = lio_pose_orig;
    }

    layers[0].init_layer_param();
    layers[0].init_storage(total_layer_num);

    // 初始化非底层的参数
    for (int i = 1; i < total_layer_num; i++)
    {
        // 上一层除开最后一个线程之外的窗口数量，这里相当于, 从第二层开始，每层的数量都是上一层的数量的1/WIN_SIZE倍
        int pose_size_ = (layers[i - 1].thread_num - 1) * layers[i - 1].part_length;

        // 加上最后一个线程的窗口数量
        pose_size_ += layers[i - 1].tail == 0 ? layers[i - 1].left_gap_num : (layers[i - 1].left_gap_num + 1);

        layers[i].init_layer_param(pose_size_);
        layers[i].init_storage(total_layer_num);
        layers[i].data_path = layers[i - 1].data_path + "process1/";
    }
    std::cout << "HBA init layer done" << std::endl;
}

void interpolate_pose(std::vector<mypcl::pose> &pose_vec_orig, std::vector<mypcl::pose> &pose_vec_tran, std::vector<double> gps_time, std::vector<double> lidar_time)
{
    std::vector<int> index_gps2lidar;
    int k = 0;
    // lidar_time.size() == lio_pose.size()
    // 解释一下，例如 j = index_gps2lidar[k]， 相当于第k个GPS点对应的lio的位姿索引为j
    for(int i = 0; i < lidar_time.size(); i++)
    {
        // 需要寻找到与当前激光点云时间最接近的GPS时间
        for(int j = k; j < gps_time.size(); j++)
        {
            if(fabs(lidar_time[i] - gps_time[j]) < 0.06 && gps_time[j] < lidar_time.back())
            {
                index_gps2lidar.push_back(i);
                k++;
                break;
            }
        }   
    }

    std::cout << "lidar gps match_point :" << index_gps2lidar.size() << std::endl;

    k = 0;
    // 找到索引之后，进行线性插值
    //std::cout << "lidar_time_size: " << lidar_time.size() << std::endl;
    //std::cout << pose_vec_tran.size() << std::endl;
    for (int j = 0; j < lidar_time.size(); j++)
    {
        // 如果找的到索引，说明此时需要对GPS点进行插值
        if (j == index_gps2lidar[k])
        {
            mypcl::pose pose_temp;
            if (gps_time[k] >= lidar_time[index_gps2lidar[k]])
            {
                //std::cout << "***11111****  k = " << k << "," << gps_time[k] << "----" <<"j =" << j << "," << lidar_time[j] << std::endl;
                pose_temp.t = pose_vec_orig[j].t + ((gps_time[k] - lidar_time[j]) * (pose_vec_orig[j + 1].t - pose_vec_orig[j].t) / (lidar_time[j + 1] - lidar_time[j]));
                pose_temp.q = pose_vec_orig[j].q.slerp((gps_time[k] - lidar_time[j]) / (lidar_time[j + 1] - lidar_time[j]), pose_vec_orig[j + 1].q);
                
                pose_vec_tran.push_back(pose_vec_orig[index_gps2lidar[k]]);
                pose_vec_tran.push_back(pose_temp);

                timestamp.push_back(lidar_time[index_gps2lidar[k]]);
                timestamp.push_back(gps_time[k]);

                index_interpolate.push_back(pose_vec_tran.size() - 1); // 记录插值后的位姿索引
            }
            else
            {
                //std::cout << "***22222****  k = " << k << "," << gps_time[k] << "----" <<"j =" << j << "," << lidar_time[j] << std::endl;
                pose_temp.t = pose_vec_orig[j - 1].t + ((gps_time[k] - lidar_time[j - 1]) * (pose_vec_orig[j].t - pose_vec_orig[j - 1].t) / (lidar_time[j] - lidar_time[j - 1]));
                pose_temp.q = pose_vec_orig[j - 1].q.slerp((gps_time[k] - lidar_time[j - 1]) / (lidar_time[j] - lidar_time[j - 1]), pose_vec_orig[j].q);

                timestamp.push_back(gps_time[k]);
                timestamp.push_back(lidar_time[index_gps2lidar[k]]);

                pose_vec_tran.push_back(pose_temp);
                index_interpolate.push_back(pose_vec_tran.size() - 1); // 记录插值后的位姿索引
                pose_vec_tran.push_back(pose_vec_orig[index_gps2lidar[k]]);
            }      
            k++;
        }
        else
        {
            pose_vec_tran.push_back(pose_vec_orig[j]);
            timestamp.push_back(lidar_time[j]);
        }
    }
    mypcl::write_pose(pose_vec_tran, data_path);
    std::cout << "k = " << k << std::endl;
    std::cout << index_interpolate.size() << std::endl;
    std::cout << " GPS Point has been interpolated to LIO Point, total_size: " << pose_vec_tran.size() << std::endl;
    
    k = 0;
    // 以上步骤完成了GPS点的pose插值，但是此时插值后的点没有对应的点云，这个时候需要根据pose进行点云的转换
    std::vector<pcl::PointCloud<PointType>::Ptr> tran_pc;
    pcl::PointCloud<PointType>::Ptr temp_pc(new pcl::PointCloud<PointType>);
    tran_pc.reserve(pose_vec_tran.size());
    pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>); // 创建点云指针

    int count = 0;
    for(int i = 0; i < pose_vec_tran.size(); i++)
    {
        if (i == index_interpolate[k])
        {
            // RCLCPP_INFO(rclcpp::get_logger("hba"), "val=%ld\n", tran_pc[i - 1]->points.size());
            pcl::PointCloud<PointType>::Ptr pc_in(new pcl::PointCloud<PointType>);
            pcl::PointCloud<PointType>::Ptr temp_pc(new pcl::PointCloud<PointType>);
            pc_in = tran_pc[i - 1];

            Eigen::Matrix3d R_from = pose_vec_tran[i - 1].q.toRotationMatrix();
            Eigen::Matrix3d R_to = pose_vec_tran[i].q.toRotationMatrix();

            Eigen::Matrix3d R_rel = R_to.transpose() * R_from;
            Eigen::Vector3d t_rel = R_to.transpose() * (pose_vec_tran[i - 1].t - pose_vec_tran[i].t);

            for(size_t i = 0; i < pc_in->points.size(); i++)
            {
                Eigen::Vector3d pt_cur(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
                Eigen::Vector3d pt_to;
                pt_to = R_rel * pt_cur + t_rel;
                temp_pc->points.push_back(pcl::PointXYZI(pt_to.x(), pt_to.y(), pt_to.z(), pc_in->points[i].intensity));
            }
            tran_pc.push_back(temp_pc);

            k++;
        }
        else
        {
            mypcl::loadPCD(data_path, pcd_name_fill_num, pc, count + 1, "pcd/");
            count++;
            tran_pc.push_back(pc);
        }
    }
    //std::cout << "count:" << count << std::endl;

    // 将pcd文件写回去
    for(int i = 0; i < pose_vec_tran.size(); i++)
    {
        mypcl::savePCD(data_path,pcd_name_fill_num, tran_pc[i], i, "pcd_trans/");
    }
    std::cout << "******All GPS Points has been add relative pcds! *****" << std::endl;
}

std::vector<mypcl::pose> write_back_pose(std::vector<mypcl::pose> lio_tran_pose)
{
    int k = 0;
    std::vector<mypcl::pose> lio_pose;
    for (int i = 0; i < lio_tran_pose.size(); i++)
    {
        if(i == index_interpolate[k])
        {
            k++;
            continue;
        }
        else
        {
            lio_pose.push_back(lio_tran_pose[i]);
        }
    }
    return lio_pose;
}


void HBA::update_next_layer_state(int cur_layer_num)
{
    // 遍历当前层的所有线程
    for (int i = 0; i < layers[cur_layer_num].thread_num; i++)
    {
        if (i < layers[cur_layer_num].thread_num - 1) // 非最后一个线程
        {
            // 遍历第i个线程的每个窗口
            for (int j = 0; j < layers[cur_layer_num].part_length; j++)
            {
                int index = (i * layers[cur_layer_num].part_length + j) * GAP; // 当前层的第i个线程的第j个窗口的索引

                // 当前层每个窗口内的第一个点的索引作为下一层的位姿
                layers[cur_layer_num + 1].pose_vec[i * layers[cur_layer_num].part_length + j] = layers[cur_layer_num].pose_vec[index];
            }
        }
        else
        {
            for (int j = 0; j < layers[cur_layer_num].j_upper; j++)
            {
                int index = (i * layers[cur_layer_num].part_length + j) * GAP;
                layers[cur_layer_num + 1].pose_vec[i * layers[cur_layer_num].part_length + j] = layers[cur_layer_num].pose_vec[index];
            }
        }
    }
}

/*
    理解：hessian是怎么来的
    首先，定义Hess是一个60*60的矩阵，分别存放了窗口内10个点的位姿的Hessian矩阵的上三角部分元素
    1. 在hba.cpp中，通过damping_iter()计算每一个线程的每个窗口的Hessian，
    1. 通过ba.hpp中的divide_thread()函数，将任务分配给每个线程，调用acc_evaluate2()计算每一个体素内的Hessian矩阵，
        就是60*60的,然后所有体素的残差，雅各布矩阵和Hessian矩阵累加得到总的Hessian矩阵，雅各布矩阵和残差。（因为Hessian具有线性累加性质，因为 Hessian 矩阵本质上描述的是二阶导数的累积特性）
    2.
    */
// 使用GTSAM进行位姿图优化
void HBA::pose_graph_optimization(GPS_Factor &gps_factor_func)
{
    // 初始化位姿和hessian矩阵
    std::vector<mypcl::pose> upper_pose, init_pose;    // 上层位姿和初始位姿(q,t)
    upper_pose = layers[total_layer_num - 1].pose_vec; // 顶层位姿
    init_pose = layers[0].pose_vec;                    // 底层位姿
    std::vector<VEC(6)> upper_cov, init_cov;           // 上层和初始的协方差矩阵
    upper_cov = layers[total_layer_num - 1].hessians;
    init_cov = layers[0].hessians;

    // 初始化gtsam
    int cnt = 0;
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Vector Vector6(6);                      // 定义一个6维向量
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8; // 初始化6维向量

    // 先验噪声模型
    gtsam::noiseModel::Diagonal::shared_ptr priorModel = gtsam::noiseModel::Diagonal::Variances(Vector6);

    // 插入底层的第一个位姿作为初始位姿
    initial.insert(0, gtsam::Pose3(gtsam::Rot3(init_pose[0].q.toRotationMatrix()), gtsam::Point3(init_pose[0].t)));

    // 添加先验因子
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(gtsam::Rot3(init_pose[0].q.toRotationMatrix()), gtsam::Point3(init_pose[0].t)), priorModel));

    // 遍历底层的位姿，添加因子到因子图中
    for (uint i = 0; i < init_pose.size(); i++)
    {
        // 逐个添加底层的位姿
        if (i > 0)
        {
            initial.insert(i, gtsam::Pose3(gtsam::Rot3(init_pose[i].q.toRotationMatrix()), gtsam::Point3(init_pose[i].t)));
        }

        // 添加每个窗口内第一个位姿的因子 i%GAP==0确保了每个窗口的第一个位姿
        if (i % GAP == 0 && cnt < init_cov.size())
        {
            // 遍历窗口中的所有hessian
            for (int j = 0; j < WIN_SIZE - 1; j++)
            {
                for (int k = j + 1; k < WIN_SIZE; k++)
                {
                    // 保证调用不越界
                    if (i + j + 1 >= init_pose.size() || i + k >= init_pose.size())
                    {
                        break;
                    }

                    cnt++; // 这里的cnt是用来计数的，用来计算Hessian矩阵的数量，也就是窗口内两两之间的Hessian矩阵的数量，比如一个窗口内就是10个位姿，那么就有10*9/2个Hessian矩阵

                    if (init_cov[cnt - 1].norm() < 1e-20)
                    {
                        continue; // 如果Hessian矩阵的范数小于1e-20（即Hessian过小），跳过
                    }

                    // 设置窗口内第j个位姿相对于第一个位姿的位移和旋转
                    Eigen::Vector3d t_ab = init_pose[i + j].t;
                    Eigen::Matrix3d R_ab = init_pose[i + j].q.toRotationMatrix();

                    // 更新为，窗口内第k个位姿相对于第j个位姿的位移和旋转, 需要这个矩阵主要是为了添加两个节点之间的约束
                    t_ab = R_ab.transpose() * (init_pose[i + k].t - t_ab);
                    R_ab = R_ab.transpose() * init_pose[i + k].q.toRotationMatrix();

                    gtsam::Rot3 R_sam(R_ab);
                    gtsam::Point3 t_sam(t_ab);

                    // 根据位姿间的hessians设置里程计噪声模型
                    Vector6 << fabs(1.0 / init_cov[cnt - 1](0)), fabs(1.0 / init_cov[cnt - 1](1)), fabs(1.0 / init_cov[cnt - 1](2)),
                        fabs(1.0 / init_cov[cnt - 1](3)), fabs(1.0 / init_cov[cnt - 1](4)), fabs(1.0 / init_cov[cnt - 1](5));
                    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

                    // 添加里程计两两位姿间的factor
                    gtsam::NonlinearFactor::shared_ptr factor(new gtsam::BetweenFactor<gtsam::Pose3>(i + j, i + k, gtsam::Pose3(R_sam, t_sam), odometryNoise));
                    graph.push_back(factor);
                }
            }
        }
    }

    // 遍历顶层的位姿，添加因子到因子图中
    int pose_size = upper_pose.size();
    cnt = 0;

    // 不按窗口添加因子，而是直接两两之间添加因子
    for (int i = 0; i < pose_size - 1; i++)
    {
        for (int j = i + 1; j < pose_size; j++)
        {
            cnt++;
            if (upper_cov[cnt - 1].norm() < 1e-20)
            {
                continue;
            }

            Eigen::Vector3d t_ab = upper_pose[i].t;
            Eigen::Matrix3d R_ab = upper_pose[i].q.toRotationMatrix();
            t_ab = R_ab.transpose() * (upper_pose[j].t - t_ab);
            R_ab = R_ab.transpose() * upper_pose[j].q.toRotationMatrix();
            gtsam::Rot3 R_sam(R_ab);
            gtsam::Point3 t_sam(t_ab);

            Vector6 << fabs(1.0 / upper_cov[cnt - 1](0)), fabs(1.0 / upper_cov[cnt - 1](1)), fabs(1.0 / upper_cov[cnt - 1](2)),
                fabs(1.0 / upper_cov[cnt - 1](3)), fabs(1.0 / upper_cov[cnt - 1](4)), fabs(1.0 / upper_cov[cnt - 1](5));
            gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
            // 这里使用pow()函数是因为在上一层的时候，每个窗口内的位姿数量是下一层的GAP倍，所以这里需要乘以GAP的total_layer_num - 1次方，这里就是要追溯到底层的位姿索引
            gtsam::NonlinearFactor::shared_ptr factor(new gtsam::BetweenFactor<gtsam::Pose3>(i * pow(GAP, total_layer_num - 1),
                                                                                             j * pow(GAP, total_layer_num - 1), gtsam::Pose3(R_sam, t_sam), odometryNoise));
            graph.push_back(factor);
        }
    }
    
    
    if (enable_gps_factor == true)
    {
        std::cout << index_interpolate.size() << std::endl;
        gps_factor_func.Add_GPS_Factor(gps_pose_vec, init_pose, graph, init_cov, index_interpolate);
    }
    
    // 此时已经完成了所有因子的添加，接下来就是使用gtsam进行优化
    gtsam::ISAM2Params parameters;
    // 重新线性化阈值,由于非线性函数的特性，随着优化的进行，误差函数可能会发生较大的变化，导致线性化的误差逐渐增大。为了避免这种误差积累影响优化过程，通常会在优化的过程中引入 重新线性化 的机制
    parameters.relinearizeThreshold = 0.01;
    // 重新线性化跳过,设置=1，确保每次迭代都会重新线性化
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2 isam(parameters); // 创建ISAM2对象
    isam.update(graph, initial);   // 引入因子图和初始值
    isam.update();                 // 进行优化

    gtsam::Values results = isam.calculateEstimate(); // 计算估计值

    cout << "vertex size" << results.size() << endl;

    for (uint i = 0; i < results.size(); i++)
    {
        // 使用results中优化后的位姿来更新初始位姿
        gtsam::Pose3 pose = results.at(i).cast<gtsam::Pose3>();
        assign_qt(init_pose[i].q, init_pose[i].t, Eigen::Quaterniond(pose.rotation().toQuaternion()), pose.translation());
    }
    std::vector<mypcl::pose> init_pose_tran = write_back_pose(init_pose);
    mypcl::write_pose(init_pose_tran, data_path); // 保存优化后的位姿
    mypcl::write_evo_pose(init_pose, data_path); // 保存优化后的位姿
    std::cout << "----------GTSAM-PGO COMPLETE!-----------" << std::endl;
}

void cut_voxel(unordered_map<VOXEL_LOC, OCTO_TREE_ROOT *> &feat_map,
               pcl::PointCloud<PointType> &feat_pt,
               Eigen::Quaterniond q, Eigen::Vector3d t, int fnum,
               double voxel_size, int window_size, float eigen_ratio)
{
    float loc_xyz[3];
    for (PointType &p_c : feat_pt.points)
    {
        Eigen::Vector3d pvec_orig(p_c.x, p_c.y, p_c.z);
        Eigen::Vector3d pvec_tran = q * pvec_orig + t;

        for (int j = 0; j < 3; j++)
        {
            loc_xyz[j] = pvec_tran[j] / voxel_size;
            if (loc_xyz[j] < 0)
                loc_xyz[j] -= 1.0;
        }

        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = feat_map.find(position);
        if (iter != feat_map.end())
        {
            iter->second->vec_orig[fnum].push_back(pvec_orig);
            iter->second->vec_tran[fnum].push_back(pvec_tran);

            iter->second->sig_orig[fnum].push(pvec_orig);
            iter->second->sig_tran[fnum].push(pvec_tran);
        }
        else // 如果未找到，创建一个新的体素树根节点，并将其添加到feat_map中
        {
            OCTO_TREE_ROOT *ot = new OCTO_TREE_ROOT(window_size, eigen_ratio);
            ot->vec_orig[fnum].push_back(pvec_orig);
            ot->vec_tran[fnum].push_back(pvec_tran);
            ot->sig_orig[fnum].push(pvec_orig);
            ot->sig_tran[fnum].push(pvec_tran);

            ot->voxel_center[0] = (0.5 + position.x) * voxel_size;
            ot->voxel_center[1] = (0.5 + position.y) * voxel_size;
            ot->voxel_center[2] = (0.5 + position.z) * voxel_size;
            ot->quater_length = voxel_size / 4.0;
            ot->layer = 0;
            feat_map[position] = ot;
        }
    }
}
void parallel_compute_tool(LAYER &layer, int thread_id, LAYER &next_layer, int i, int win_size)
{
    double t_temp = 0;
    //cout << "Current thread id: " << thread_id + 1 << " layer num: " << layer.layer_num << endl;
    // raw_pc为当前窗口原始的点云，src_pc为降采样+转换+合并后的点云
    vector<pcl::PointCloud<PointType>::Ptr> src_pc, raw_pc;
    src_pc.resize(win_size);
    raw_pc.resize(win_size);

    double residual_cur = 0, residual_pre = 0; // 当前残差，上一次残差
    vector<IMUST> x_buf(win_size);             // 状态量

    // 计算每个窗口内的位姿
    for (int j = 0; j < win_size; j++)
    {
        x_buf[j].R = layer.pose_vec[i * GAP + j].q.toRotationMatrix();
        x_buf[j].p = layer.pose_vec[i * GAP + j].t;
    }
    
    if (layer.layer_num != 1) // 非底层
    {
        t_temp = rclcpp::Clock().now().seconds();
        for (int j = i * GAP; j < i * GAP + win_size; j++)
        {
            src_pc[j - i * GAP] = (*layer.pcds[j]).makeShared();
        }
        //cout << "load pcd time: " << rclcpp::Clock().now().seconds() - t_temp << endl;
    }
    size_t mem_cost = 0;
    for (int loop = 0; loop < layer.max_iter; loop++)
    {
        if (layer.layer_num == 1)
        {
            t_temp = rclcpp::Clock().now().seconds();
            // 从路径中读取点云pcd文件, 逐窗口读取
            for (int j = i * GAP; j < i * GAP + win_size; j++)
            {
                if (loop == 0)
                {

                    pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>); // 创建点云指针
                    mypcl::loadPCD(layer.data_path, pcd_name_fill_num, pc, j, "pcd_trans/");
                    raw_pc[j - i * GAP] = pc;
                }
                src_pc[j - i * GAP] = (*raw_pc[j - i * GAP]).makeShared();
            }
            //cout << "load pcd time: " << rclcpp::Clock().now().seconds() - t_temp << endl;
        }

        // 建立体素地图
        unordered_map<VOXEL_LOC, OCTO_TREE_ROOT *> surf_map;

        // 对该窗口进行体素降采样并划分这个窗口的根节点体素
        for (size_t j = 0; j < win_size; j++)
        {
            if (layer.downsample_size > 0)
            {
                t_temp = rclcpp::Clock().now().seconds();
                downsample_voxel(*src_pc[j], layer.downsample_size);
                //cout << "downsample time: " << rclcpp::Clock().now().seconds() - t_temp << endl;
                t_temp = 0;

            }
            cut_voxel(surf_map, *src_pc[j], Eigen::Quaterniond(x_buf[j].R), x_buf[j].p, j, layer.voxel_size, win_size, layer.eigen_ratio);
            //cout << "cut voxel time: " << rclcpp::Clock().now().seconds() - t_temp << endl;
        }

        t_temp = rclcpp::Clock().now().seconds();
        // 继续划分体素，确定体素的特征，完成体素地图的建立
        for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        {
            iter->second->recut();
        }
        //cout << "recut time: " << rclcpp::Clock().now().seconds() - t_temp << endl;

        // 进行优化过程
        VOX_HESS voxhess(win_size);
        t_temp = rclcpp::Clock().now().seconds();
        for (auto iter = surf_map.begin(); iter != surf_map.end(); iter++)
        {
            // 向体素容器中添加体素的特征、点云数据(ifdef ENABLE_FILTER)
            iter->second->tras_opt(voxhess);
        }
        //cout << "tras_opt time: " << rclcpp::Clock().now().seconds() - t_temp << endl;

        VOX_OPTIMIZER opt_lsv(win_size);
        t_temp = rclcpp::Clock().now().seconds();
        // 去除该窗口内的离群点，一个窗口内去除最多ratio比例的离群点
        opt_lsv.remove_outlier(x_buf, voxhess, layer.reject_ratio);
        //cout << "remove outlier time: " << rclcpp::Clock().now().seconds() - t_temp << endl;
        PLV(6)
        hess_vec;

        // 窗口内进行阻尼迭代
        t_temp = rclcpp::Clock().now().seconds();
        opt_lsv.damping_iter(x_buf, voxhess, residual_cur, hess_vec, mem_cost);
        //cout << "damping_iter time: " << rclcpp::Clock().now().seconds() - t_temp << endl;

        for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        {
            // 释放体素地图的内存，其实是只删除了second，也就是octo_tree_root的部分

            delete iter->second;
        }

        // 更新hess和内存占用量
        if (loop > 0 && abs(residual_pre - residual_cur) / abs(residual_cur) < 0.05 || loop == layer.max_iter - 1)
        {
            // 将内存占用更新为当前的最大值
            if (layer.mem_costs[thread_id] < mem_cost)
            {
                layer.mem_costs[thread_id] = mem_cost;
            }

            if (win_size == WIN_SIZE)
            {
                // 更新hessians, 一个窗口内有WIN_SIZE * (WIN_SIZE - 1) / 2个hessian矩阵
                for (int j = 0; j < win_size * (win_size - 1) / 2; j++)
                {
                    // 存储hessian矩阵
                    layer.hessians[i * (win_size - 1) * win_size / 2 + j] = hess_vec[j];
                }
            }
            else
            {
                for (int j = 0; j < win_size * (win_size - 1) / 2; j++)
                {
                    layer.hessians[i * (WIN_SIZE - 1) * WIN_SIZE / 2 + j] = hess_vec[j];
                }
            }

            break;
        }
        residual_pre = residual_cur; // 更新上一次的残差
    }
    //cout << thread_id + 1<<"  thread's" <<"Layer  " << layer.layer_num << "  's residual = "<< residual_cur <<endl;
    pcl::PointCloud<PointType>::Ptr pc_keyframe(new pcl::PointCloud<PointType>); // 一个窗口的总点云
    for (size_t j = 0; j < win_size; j++)
    {
        Eigen::Quaterniond q_tmp;
        Eigen::Vector3d t_tmp;
        // 将x_buf中的R和t，先转到窗口内的第一个位姿的坐标系下后，赋值给q_tmp和t_tmp, 这里的.inverse()等价于.transpose()
        assign_qt(q_tmp, t_tmp, Eigen::Quaterniond(x_buf[0].R.inverse() * x_buf[j].R), x_buf[0].R.inverse() * (x_buf[j].p - x_buf[0].p));

        // 一个位姿对应的点云
        pcl::PointCloud<PointType>::Ptr pc_oneframe(new pcl::PointCloud<PointType>);

        // 将点云src_pc按照q_tmp和t_tmp进行变换,也就是把每个窗口的每个位姿的点云转换到第一个位姿的坐标系下，结果存储在pc_oneframe中
        mypcl::transform_pointcloud(*src_pc[j], *pc_oneframe, t_tmp, q_tmp);
        pc_keyframe = mypcl::append_cloud(pc_keyframe, *pc_oneframe); // 将每一个位姿对应的点云合并到窗口内的点云
    }
    downsample_voxel(*pc_keyframe, 0.05); // 体素降采样
    // 这个窗口内的点云赋值给下一层的pcd，直观来讲，就是将这个窗口的所有点云集中到第一个位姿的点云中，因为下一层只取这层的每个窗口的第一个位姿
    next_layer.pcds[i] = pc_keyframe;
    //cout << "Current task complete, thread_id = "<< thread_id + 1 << "layer = "<< layer.layer_num << endl;
}

// 执行某层非最后一个线程的并行计算
void parallel_head(LAYER &layer, int thread_id, LAYER &next_layer)
{
    int &part_length = layer.part_length;
    int &layer_num = layer.layer_num;

    // 处理当前层每一个线程的任务，也就是处理每个窗口
    for (int i = thread_id * part_length; i < (thread_id + 1) * part_length; i++)
    {
        parallel_compute_tool(layer, thread_id, next_layer, i, WIN_SIZE);
    }
}

// 执行最后一个线程的并行计算
void parallel_tail(LAYER &layer, int thread_id, LAYER &next_layer)
{
    int &part_length = layer.part_length;
    int &layer_num = layer.layer_num;
    int &left_gap_num = layer.left_gap_num;

    // 记录各种事件的时间
    // double load_t = 0, undis_t = 0, dsp_t = 0, cut_t = 0, recut_t = 0, total_t = 0, tran_t = 0, sol_t = 0, save_t = 0;

    if (layer.gap_num - (layer.thread_num - 1) * part_length + 1 != left_gap_num)
    {
        std::cout<< "This layer's left_gap_num is wrong!" << endl;
    }

    // 处理最后一个线程的满窗口
    for (uint i = thread_id * part_length; i < thread_id * part_length + left_gap_num; i++)
    {
        parallel_compute_tool(layer, thread_id, next_layer, i, WIN_SIZE);
    }

    if (layer.tail > 0)
    {
        int i = thread_id * part_length + left_gap_num;
        parallel_compute_tool(layer, thread_id, next_layer, i, layer.last_win_size);
    }
}

void global_ba(LAYER &layer)
{
    int window_size = layer.pose_vec.size();
    vector<IMUST> x_buf(window_size);
    for (int i = 0; i < window_size; i++)
    {
        x_buf[i].R = layer.pose_vec[i].q.toRotationMatrix();
        x_buf[i].p = layer.pose_vec[i].t;
    }

    vector<pcl::PointCloud<PointType>::Ptr> src_pc;
    src_pc.resize(window_size);
    for (int i = 0; i < window_size; i++)
    {
        src_pc[i] = (*layer.pcds[i]).makeShared();
    }

    double residual_cur = 0, residual_pre = 0;
    size_t mem_cost = 0, max_mem = 0;

    std::cout << "---------------------" << std::endl;
    std::cout << "Global BA Iteration Start:" << std::endl;
    for (int loop = 0; loop < layer.max_iter; loop++)
    {
        std::cout << "---------------------" << std::endl;
        std::cout << "Iteration " << loop << std::endl;

        unordered_map<VOXEL_LOC, OCTO_TREE_ROOT *> surf_map;

        for (int i = 0; i < window_size; i++)
        {
            if (layer.downsample_size > 0)
            {
                downsample_voxel(*src_pc[i], layer.downsample_size);
            }
            cut_voxel(surf_map, *src_pc[i], Eigen::Quaterniond(x_buf[i].R), x_buf[i].p, i, layer.voxel_size, window_size, layer.eigen_ratio);
        }

        for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        {
            iter->second->recut();
        }

        VOX_HESS voxhess(window_size);
        for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        {
            iter->second->tras_opt(voxhess);
        }

        VOX_OPTIMIZER opt_lsv(window_size);
        opt_lsv.remove_outlier(x_buf, voxhess, layer.reject_ratio);
        PLV(6)
        hess_vec;
        opt_lsv.damping_iter(x_buf, voxhess, residual_cur, hess_vec, mem_cost);

        for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        {
            delete iter->second;
        }

        cout << "Residual absolute: " << abs(residual_pre - residual_cur) << " | "
             << "percentage: " << abs(residual_pre - residual_cur) / abs(residual_cur) << endl;

        if (loop > 0 && abs(residual_pre - residual_cur) / abs(residual_cur) < 0.05 || loop == layer.max_iter - 1)
        {
            if (mem_cost > max_mem)
            {
                max_mem = mem_cost;
            }
#ifdef FULL_HESS
            for (int i = 0; i < window_size * (window_size - 1) / 2; i++)
            {
                layer.hessians[i] = hess_vec[i];
            }
#else
            for (int i = 0; i < window_size - 1; i++)
            {
                Matrix6d hess = Hess_cur.block(6 * i, 6 * i + 6, 6, 6);
                for (int row = 0; row < 6; row++)
                {
                    for (int col = 0; col < 6; col++)
                    {
                        hessFile << hess(row, col) << ((row * col == 25) ? "" : " ");
                    }
                }
                if (i < window_size - 2)
                {
                    hessFile << "\n";
                }
            }
#endif
            break;
        }
        residual_pre = residual_cur;
    }
    // 这里不知道为什么只有顶层BA会更新pose_vec, 后面试试前面的也更新
    for (int i = 0; i < window_size; i++)
    {
        layer.pose_vec[i].q = Quaterniond(x_buf[i].R);
        layer.pose_vec[i].t = x_buf[i].p;
    }
}

void distribute_thread(LAYER &layer, LAYER &next_layer)
{
    int &thread_num = layer.thread_num;
    // 创建线程任务
    for (int i = 0; i < thread_num; i++)
    {
        if (i < thread_num - 1)
        {
            layer.mthreads[i] = new thread(parallel_head, ref(layer), i, ref(next_layer));
        }
        else
        {
            layer.mthreads[i] = new thread(parallel_tail, ref(layer), i, ref(next_layer));
        }
    }

    // 分线程处理
    for (int i = 0; i < thread_num; i++)
    {
        layer.mthreads[i]->join(); // 这会阻塞当前线程，直到 mthreads[i] 指向的线程完成执行。
        delete layer.mthreads[i];  // 在等待线程完成后，使用 delete 释放 mthreads[i] 指向的线程对象的内存。这是为了避免内存泄漏。
    }
}

class HBA_Node : public rclcpp::Node
{
public:
    HBA_Node(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("hba_node", options)
    {
        this->declare_parameter("total_layer_num", 3);
        this->declare_parameter("pcd_name_fill_num", 4);
        this->declare_parameter("data_path", "/home/jhua/hba_data/avia1/");
        this->declare_parameter("thread_num", 16);
        this->declare_parameter("enable_gps_factor", true);

        this->get_parameter("total_layer_num", total_layer_num);
        this->get_parameter("pcd_name_fill_num", pcd_name_fill_num);
        this->get_parameter("data_path", data_path);
        this->get_parameter("thread_num", thread_num);
        this->get_parameter("enable_gps_factor", enable_gps_factor);
        GPS_Factor gps_factor_func;
        HBA hba(total_layer_num, data_path, thread_num, gps_factor_func);

        for (int i = 0; i < total_layer_num - 1; i++)
        {
            std::cout << "----------Process No."<< i + 1 <<" Layer-----------" << std::endl;
            distribute_thread(hba.layers[i], hba.layers[i + 1]);
            hba.update_next_layer_state(i);
            std::cout << "-------------- Done -----------" << std::endl;
        }

        // 全局BA
        global_ba(hba.layers[total_layer_num - 1]);
        mypcl::write_interpolate_pose(hba.layers[0].pose_vec, data_path);
        hba.pose_graph_optimization(gps_factor_func);

        std::cout << "----------HBA Iteration Complete!-----------" << std::endl;
    }
    ~HBA_Node() {}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HBA_Node>());
    rclcpp::shutdown();

    return 0;
}