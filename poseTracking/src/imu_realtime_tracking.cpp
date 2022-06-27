
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h> 
#include <Eigen/Core>
#include <Eigen/Dense>
#include "manif/SE3.h"
#include "manif/impl/lie_group_base.h"
#define PI (3.1415926535897932346f)

#include "udp_reciever.h"

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
}

// 1. 输入数据信息
// 2. 姿态信息
// 3. 速度信息
// 4. 加速度信息

void data_interpreter(imuDataPack iData, manif::SE3d &pose, manif::SE3Tangentd &vel, manif::SE3Tangentd &acc)
{
    // 获取角度
    double psi = iData.angle[0] * PI / 180.;
    //来自产品手册 产品测量角度是欧拉角的，欧拉角的旋转顺序书zyx顺序
    
    double theta = iData.angle[1] * PI / 180.;
    double phi = iData.angle[2] * PI / 180.;

    cout << iData.angle[0] << ", " << iData.angle[1] << ", " << iData.angle[2] << endl;
    //// 姿态
    Eigen::Matrix3d poseEi = 
        (Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitX()).matrix() *
        Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).matrix() *
        Eigen::AngleAxisd (phi, Eigen::Vector3d::UnitZ()).matrix());

    //cout << Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()).matrix() << endl;
    
    cout << poseEi << endl << endl;;

    // 计算角速度
    Eigen::Matrix3d eul2Avel_m;
    eul2Avel_m << 1, 0, -sin(theta),
        0, cos(psi), sin(psi)* cos(theta),
        0, -sin(psi), cos(psi)* cos(theta);
                
    // 计算
    Eigen::Vector3d eulVel = Eigen::Vector3d(iData.w[0], iData.w[1], iData.w[2]);
    Eigen::Vector3d Avel = eul2Avel_m * eulVel;

    Eigen::Isometry3d poseIso = Eigen::Isometry3d::Identity();
    poseIso.linear() = poseEi;
    pose = manif::SE3d(poseIso);

    //// 计算角速度
    vel.setZero();
    vel.ang() = Avel;
}

int main()
{
    // 1. 初始化 机器人姿态信息
    Eigen::Isometry3d Tri = Eigen::Isometry3d::Identity();
    manif::SE3d se3_pos(Tri);

    // 保存的姿态轨迹信息
    std::vector<manif::SE3d> se3_pos_v;
    std::vector<manif::SE3d> se3_tarpose_v;


    // 空置率参数
    double k_p = 1;
    // 时间间隔
    double delta_T = 0.04;

    // 对角速度，角加速度进行约束
    double rot_vec_bound = 0.6;
    double rot_acc_bound = 3;

    // 使用windows记录时刻信息

    // 初始化轨迹上一时刻的角速度矢量
    manif::SE3Tangentd se3_vel_t_1 = manif::SE3Tangentd::Zero();

    // 2. 初始化硬件通讯
    imu_com imu;

    // 精准计算时间

    // 3. 主循环使用控制率，并记录跟踪信息
    double start, end;
    start = clock();

    imuDataPack iData;

    int iter = 0;
    while (imu.get_imu_data(iData) && iter < 1000)
    {
        iter++;
        // TODO：对得到的位姿信息进行处理
        end = clock();
        delta_T = end - start;
        start = end;

        // TODO:设置本周期目标位姿（使用真实位置替换）
        manif::SE3d pose;
        manif::SE3Tangentd vel;
        manif::SE3Tangentd acc;
        data_interpreter(iData, pose, vel, acc);

        manif::SE3d tar_pose = pose;

        // mc = ma.rplus(mb.rminus(ma) * t);

        // TODO:设置本周期目标速度（使用真实速度替换）
        manif::SE3Tangentd tar_vel = vel;

        // 计算控制率产生速度
        manif::SE3Tangentd se3_vel_cLaw = tar_pose.rminus(se3_pos) * k_p + tar_vel;

        // 计算控制率产生的角加速度（替换实际时间，查）
        manif::SE3Tangentd se3_acc = (se3_vel_cLaw - se3_vel_t_1) / delta_T;

        // 如果加速度过大，对整个矢量进行截断
        manif::SE3Tangentd applied_vel = se3_vel_cLaw;
        if (se3_acc.ang().norm() > rot_acc_bound)
        {
            manif::SE3Tangentd applied_acc = se3_acc / se3_acc.ang().norm() * rot_acc_bound;
            //manif::SE2Tangentd applied_acc = manif::SE2Tangentd(se2_acc.x(), se2_acc.y(), sign(se2_acc.angle()) * rot_acc_bound);
            applied_vel = se3_vel_t_1 + applied_acc * delta_T;
        }
        // 如果速度过大，约束速度
        if (applied_vel.ang().norm() > rot_vec_bound)
        {
            applied_vel = applied_vel / applied_vel.ang().norm() * rot_vec_bound;
        }

        // 更新位置
        se3_pos = se3_pos.rplus(applied_vel * delta_T);

        // 保存轨迹信息
        se3_pos_v.push_back(se3_pos);
        se3_tarpose_v.push_back(tar_pose);

        // 保存速度矢量
        se3_vel_t_1 = applied_vel;
    }

    // 使用windows记录时刻信息
    std::string fileName = "SE3_P_control_path";
    save(fileName, se3_pos_v);

    // 使用windows记录时刻信息
    std::string imufileName = "SE3_imuData";
    save(imufileName, se3_tarpose_v);

    system("pause");
    return 0;
}


