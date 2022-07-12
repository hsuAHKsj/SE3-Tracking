
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h> 
#include <Eigen/Core>
#include <Eigen/Dense>

#include <thread>
#include <mutex>
#include "manif/SE3.h"
#include "manif/impl/lie_group_base.h"
#include "udp_reciever.h"

#define PI (3.1415926535897932346f)
#define RECORD_TIME 300

std::mutex m;
bool returnFlag;

imuDataPack iData;
typedef vector<double> timeStamps;


// 独立线程读取并保存位置信息
void imu_record_pose_info()
{
    imu_com imu;

    //std::lock_guard<std::mutex> lockGuard(m);
    // 保存imu信息，并存到全局变量里面
    imuDataPack iData_temp;

    while (returnFlag == false)
    {
        imu.get_imu_data(iData_temp);
        m.lock();
        iData = iData_temp;
        m.unlock();
    }

    return;
}

void save(std::string fileName, std::vector<manif::SE3d> SE3_path);

void saveTime(std::string fileName, timeStamps time);

void data_interpreter(const imuDataPack& iData, manif::SE3d& pose)
{
    // 获取角度
    double psi = iData.angle[0] * PI / 180.;
    //来自产品手册 产品测量角度是欧拉角的，欧拉角的旋转顺序书zyx顺序

    double theta = iData.angle[1] * PI / 180.;
    double phi = iData.angle[2] * PI / 180.;

    cout << " IMU Data : " << iData.angle[0] << ", " << iData.angle[1] << ", " << iData.angle[2] << endl;

    // 姿态
    Eigen::Matrix3d poseEi =
        (Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitX()).matrix() *
            Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).matrix() *
            Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()).matrix());

    Eigen::Isometry3d poseIso = Eigen::Isometry3d::Identity();
    poseIso.linear() = poseEi;
    pose = manif::SE3d(poseIso);
}

int main()
{
    //////////////// 数据容器 /////////////////
    // 保存的姿态轨迹信息
    std::vector<manif::SE3d> se3_pos_v;
    std::vector<manif::SE3d> se3_tarpose_v;
    std::vector<double> time_v;
    std::vector<double> T_v;

    //////////////// 基础参数信息/////////////////
    // 空置率参数
    double k_p = 1;
    // 时间间隔
    double delta_T = 0.04; // 2ms更新一次

    // 对角速度，角加速度进行约束
    double rot_vec_bound = 0.6;
    double rot_acc_bound = 3;

    //////////////// 数据采集线程 /////////////////

    // 开启imu线程,数据开始被采集
    thread thread_IMU(imu_record_pose_info);
    returnFlag = false;

    //////////////// 机器人初始状态 /////////////////

    // 1. 机器人初始状态
    Eigen::Isometry3d Tri = Eigen::Isometry3d::Identity();
    manif::SE3d se3_pos(Tri);

    // 2. 动态系统上一时刻速度
    manif::SE3Tangentd se3_vel_t_1 = manif::SE3Tangentd::Zero();
    // 保存上一周期的目标速度
    manif::SE3Tangentd se3_Tarvel_t_1 = manif::SE3Tangentd::Zero();
    // 保存上一周期的加速度
    manif::SE3Tangentd se3_acc_t_1 = manif::SE3Tangentd::Zero();

    //////////////// 动态系统更新 /////////////////

    int iter = 0;

    bool init_flag = false;

    clock_t start, finish;
    start = clock();
    double Times;

    double Sleep_delta_T = 0.01;
    while (iter++ < RECORD_TIME)
    {
        //// 本周期的状态 ////
        manif::SE3d pose;

        m.lock();
        imuDataPack iData_temp = iData;
        m.unlock();
        data_interpreter(iData_temp, pose); // 从内存池中获取

        //// 避免初状态异常 ////

        // 拿到第一个可用数据，我们才开始初始化流
        if (init_flag == false && pose.isApprox(manif::SE3d::Identity()))
        {
            Sleep(Sleep_delta_T * 1000); // 睡眠2ms
            continue;
        }
        else
        {
            init_flag = true;
        }

        //// 本周期目标状态 ////
        manif::SE3d tar_pose = pose;
        manif::SE3Tangentd tar_vel;

           // 记录时间


        // 如果池子里没有上一时刻位置，则说明在初时刻，速度设置为0；如果不在初时刻求解速度
        if (se3_tarpose_v.size() == 0)
        {
            tar_vel = manif::SE3Tangentd::Zero();
        }
        else
        {
            // 如果数据跟上一时刻数据非常接近，我们使用0 加速度的惯性运动
            // 如果位置差得非常远，用以下方法计算的目标也很有可能是不正常的
            if (tar_pose.isApprox(*se3_tarpose_v.rbegin()))
            {
                tar_vel = se3_Tarvel_t_1;
            }
            else
            {
                tar_vel = (tar_pose.rminus(*se3_tarpose_v.rbegin())) / delta_T;
            }
        }
        ////////////// 控制率编写 ///////////////////
        // 计算控制率产生速度
        manif::SE3Tangentd se3_vel_cLaw = tar_pose.rminus(se3_pos) * (k_p) + tar_vel;

        // 计算控制率产生的角加速度
        manif::SE3Tangentd se3_acc_claw = (se3_vel_cLaw - se3_vel_t_1) / delta_T;

        // 计算控制率产生的加加速度
        manif::SE3Tangentd se3_jerk = (se3_acc_claw - se3_acc_t_1) / delta_T;

        // 对角速度，角加速度进行约束
        double rot_jerk_bound = 30;

        //// 如果加速度过大，对整个矢量进行截断
        manif::SE3Tangentd applied_acc = se3_acc_claw;

        if (se3_jerk.ang().norm() > rot_jerk_bound)
        {
            manif::SE3Tangentd applied_jerk = se3_jerk / (se3_jerk.ang().norm()) * rot_jerk_bound;

            applied_acc = se3_acc_t_1 + applied_jerk * delta_T;
        }
        manif::SE3Tangentd applied_vel = se3_vel_t_1 + applied_acc * delta_T;
        if (applied_acc.ang().norm() > rot_acc_bound)
        {
            //double coeffi = 1;
            //// 如果临近边界
            //if (applied_vel.ang().norm() > 0.95 * rot_vec_bound)
            //{
            //    // 我们抑制将加速度抑制到0
            //    coeffi = ((applied_vel.ang().norm() - 0.95 * rot_vec_bound) / (0.05 * rot_vec_bound));
            //}

            applied_acc = applied_acc / applied_acc.ang().norm() * rot_acc_bound;
            applied_vel = se3_vel_t_1 + applied_acc * delta_T;
        }
        double coeffi = 1;
        // 如果速度大于临界值，且加速度使得下一周期速度的模变大，那么这一周期计算出来的加速度的模长就要逐渐减小直到零。
        if ((applied_vel.ang().norm() > 0.95 * rot_vec_bound) && ((applied_acc.ang() / applied_acc.ang().norm()).dot(se3_vel_t_1.ang() / se3_vel_t_1.ang().norm()) < 1))
        {
            if ((applied_vel.ang().norm() >= rot_vec_bound))
            {
                applied_vel = applied_vel / applied_vel.ang().norm() * rot_vec_bound;
            }
            else
            {   // 我们将加速度抑制到0
                double temp_coeff = 1 - ((applied_vel.ang().norm() - 0.95 * rot_vec_bound) / (0.05 * rot_vec_bound));
                applied_acc.ang() = applied_acc.ang() * (temp_coeff <= 0 ? 0 : temp_coeff);
                applied_vel = se3_vel_t_1 + applied_acc * delta_T;
            }
        }

        // 更新机器人位置
        se3_pos = se3_pos.rplus(applied_vel * delta_T);

        // 保存轨迹信息
        se3_pos_v.push_back(se3_pos);
        se3_tarpose_v.push_back(tar_pose);
        time_v.push_back((clock() - start) / 1000.);

        // 保存实际的加速度信息
        se3_acc_t_1 = (applied_vel - se3_vel_t_1) / delta_T;

        // 保存速度矢量
        se3_vel_t_1 = applied_vel;
        se3_Tarvel_t_1 = tar_vel;

        Sleep(Sleep_delta_T * 1000); // 睡眠2ms

    }

    // 使用windows记录时刻信息
    std::string imufileName = "SE3_imuData";
    save(imufileName, se3_tarpose_v);

    // 使用windows记录时刻信息
    std::string fileName = "SE3_P_control_path";
    save(fileName, se3_pos_v);

    //// 保存时间序列
    std::string timefileName = "Time_T";
    saveTime(timefileName, time_v);

    returnFlag = true;

    thread_IMU.join();

    system("pause");
    return 0;
}


void save(std::string fileName, std::vector<manif::SE3d> SE3_path)
{
    // 写文件，把轨迹保存起来
    using namespace std;
    // 打开并写文件
    ofstream oFileM;

    std::string pathName = "./data/" + fileName + ".csv";
    oFileM.open(pathName, ios::out | ios::trunc);

    for (const auto& interp : SE3_path)
    {
        oFileM << interp.x() << ","
            << interp.y() << ","
            << interp.z() << ","
            << interp.coeffs()(3) << ","
            << interp.coeffs()(4) << ","
            << interp.coeffs()(5) << ","
            << interp.coeffs()(6) << "\n";
    }
    oFileM.close();
}

void saveTime(std::string fileName, timeStamps time)
{
    // 写文件，把轨迹保存起来
    using namespace std;
    // 打开并写文件
    ofstream oFileM;

    std::string pathName = "./data/" + fileName + ".csv";
    oFileM.open(pathName, ios::out | ios::trunc);

    for (int i = 0; i < time.size(); i++)
    {
        oFileM << time.at(i) << "\n";
    }
}