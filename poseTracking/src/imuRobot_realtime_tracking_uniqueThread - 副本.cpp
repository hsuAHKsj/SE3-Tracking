
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

#include <hansDriver/conciseController.h>

#define PI (3.1415926535897932346f)
#define RECORD_TIME 300

struct carKinInfo
{
public:
    EcRealVector pcs;
    double vel;
    double acc;
    double jerk;
};


std::mutex m;
std::mutex robot;

bool returnFlag;

imuDataPack iData;
carKinInfo kinSharedInfo;
EcRealVector initPcsPos;
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
        Sleep(0.0001);
    }

    return;
}

void savePath(std::string fileName, std::vector<manif::SE3d> SE3_path, timeStamps time);
void saveHardwarePath(std::string fileName, std::vector<manif::SE3d> SE3_path, vector<double> ang_vel, timeStamps time);
void saveTime(std::string fileName, timeStamps time);

void save_time(std::string fileName, std::vector<double>& T_v)
{
    // 写文件，把轨迹保存起来
    using namespace std;
    // 打开并写文件
    ofstream oFileM;

    std::string pathName = "./data/" + fileName + ".csv";
    oFileM.open(pathName, ios::out | ios::trunc);

    for (const auto& Time : T_v)
    {
        oFileM << Time << "\n";
    }
}


void data_interpreter(const imuDataPack& iData, manif::SE3d& pose)
{
    // 获取角度
    double psi = iData.angle[0] * PI / 180.;
    //来自产品手册 产品测量角度是欧拉角的，欧拉角的旋转顺序书zyx顺序

    double theta = iData.angle[1] * PI / 180.;
    double phi = iData.angle[2] * PI / 180.;

    //cout << " IMU Data : " << iData.angle[0] << ", " << iData.angle[1] << ", " << iData.angle[2] << endl;

    // 姿态
    Eigen::Matrix3d poseEi =
        (Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitX()).matrix() *
            Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).matrix() *
            Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()).matrix());

    Eigen::Isometry3d poseIso = Eigen::Isometry3d::Identity();
    poseIso.linear() = poseEi;
    pose = manif::SE3d(poseIso);
}


// Gives back a rotation matrix specified with RPY convention
Eigen::Vector3d GetRPY(Eigen::Isometry3d iso)
{
    Eigen::Matrix3d data = iso.linear();
    Eigen::Vector3d outVec;
    double epsilon = 1E-12;
    outVec(1) = atan2(-data(2,0), sqrt(data(0,0) * data(0,0) + data(1,0) * data(1,0)));
    if (fabs(outVec(1)) > (PI / 2.0 - epsilon)) {
        outVec(2) = atan2(-data(0,1), data(1,1));
        outVec(0) = 0.0;
    }
    else {
        outVec(0) = atan2(data(2,1), data(2,2));
        outVec(2) = atan2(data(1,0), data(0,0));
    }

    return outVec;
}

int main()
{
    //////////////// 数据容器 /////////////////
    // 保存的姿态轨迹信息
    std::vector<manif::SE3d> se3_pos_conLaw_v;
    std::vector<manif::SE3d> se3_imu_v;
    std::vector<manif::SE3d> se3_pos_Robot_v;
    
    std::vector<double> ang_vel_v;
    std::vector<double> Time_v;
    std::vector<double> interpert_v;

    //////////////// 基础参数信息/////////////////
    // 空置率参数
    double k_p = 1;

    // 对角速度，角加速度进行约束
    double rot_vec_bound = PI;
    double rot_acc_bound = 3;
    double rot_jerk_bound = 10;

    //////////////// 数据采集线程 /////////////////
    
    // 开启imu线程,数据开始被采集
    thread thread_IMU(imu_record_pose_info);
   
    /////////////// 连接机器人，运动到初始位姿 ///////////////
    HRIF_Connect(0, "10.0.0.27", 10003);
    // 新建控制
    conciseRobotController elfin;
    // 先运动到指定目标位置
    EcRealVector initJoint = {0, 0, 90, 0, 90, 0};
    elfin.moveJ(initJoint, 360, 50);
    
    /////////////// 获取IMU和机器人的初始位姿，并计算补偿位姿 ///////////////
    elfin.readPcsPos(initPcsPos);
    manif::SE3d robInitPose(0, 0, 0, initPcsPos[3] * PI / 180, initPcsPos[4] * PI / 180, initPcsPos[5] * PI / 180);

    // 获取imu初始位姿
    manif::SE3d imuInitPose;
    do
    {
        m.lock();
        imuDataPack temImuData = iData;
        m.unlock();
        // 从内存池中获取
        data_interpreter(temImuData, imuInitPose);
        Sleep(1000);
    } while (imuInitPose.isApprox(manif::SE3d::Identity()));

    // 计算imu目标补偿位姿
    manif::SE3d imuCompensate =  imuInitPose.inverse()* robInitPose;
    Eigen::Vector3d rpyInitCompensated = GetRPY((imuInitPose * imuCompensate).isometry());

    /////////////// 初始化控制器循环中的变量 ///////////////
    
    // 1. 等待ServoP发送的指令信息
    EcRealVector poseWaitForSend = { initPcsPos[0], initPcsPos[1], initPcsPos[2], rpyInitCompensated(0) * 180 / PI, rpyInitCompensated(1) * 180 / PI, rpyInitCompensated(2) * 180 / PI };
    // 2. 控制器内部的当前信息
    manif::SE3d se3_pos(imuInitPose);
    // 3. IMU获取的目标状态
    manif::SE3d tar_pose;
    // 4. 实体机器人监听得到的状态
    EcRealVector robo_pose;

    //////////////// 上一时刻的信息状态信息 /////////////////
    // 4. 动态系统上一时刻速度
    manif::SE3Tangentd se3_vel_t_1 = manif::SE3Tangentd::Zero();
    // 5. 保存上一周期的目标速度
    manif::SE3Tangentd se3_Tarvel_t_1 = manif::SE3Tangentd::Zero();
    // 6. 保存上一周期的加速度
    manif::SE3Tangentd se3_acc_t_1 = manif::SE3Tangentd::Zero();
    // 7. 目标速度
    manif::SE3Tangentd imu_vel;

    // 开始伺服控制
    double indServoTime = 0.05;
    double delta_T = indServoTime; // 2ms更新一次
    double indLookaheadTime = 0.05;
    returnFlag = false;
    int iter = 0;

    // 开启servo指令并开始计时
    elfin.start_servo(indServoTime, indLookaheadTime);
    double begin = clock();

    //////////////// 动态系统更新 /////////////////
    while (iter++ < RECORD_TIME)
    {
        //////////////// 下发 servoP 指令 /////////////////

        double count_tick = clock();
        cout << elfin.servoP(poseWaitForSend) << endl;
        elfin.readPcsPos(robo_pose);

        interpert_v.push_back((clock() - count_tick) / 1000.);
        Time_v.push_back((clock() - begin) / 1000.);

        //////////////// 动态系统更新 /////////////////
        m.lock();
        imuDataPack iData_temp = iData;
        m.unlock();
        data_interpreter(iData_temp, tar_pose);

        //////////////// 系统初状态初始化 /////////////////
        // 如果池子里没有上一时刻位置，则说明在初时刻，速度设置为0；如果不在初时刻求解速度
        if (se3_imu_v.size() == 0)
        {
            imu_vel = manif::SE3Tangentd::Zero();
        }
        else
        {
            // 如果数据跟上一时刻数据非常接近，我们使用0 加速度的惯性运动
            // 如果位置差得非常远，用以下方法计算的目标也很有可能是不正常的

            if (tar_pose.isApprox(*se3_imu_v.rbegin()))
            {
                imu_vel = se3_Tarvel_t_1;
            }
            else
            {
                imu_vel = (tar_pose.rminus(*se3_imu_v.rbegin())) / delta_T;
            }
        }

        ////////////// 控制率编写 ///////////////////
        // 计算控制率产生速度
        manif::SE3Tangentd se3_vel_cLaw = tar_pose.rminus(se3_pos) * k_p + imu_vel;
        // 计算控制率产生的角加速度
        manif::SE3Tangentd se3_acc_claw = (se3_vel_cLaw - se3_vel_t_1) / delta_T;
        // 计算控制率产生的加加速度
        manif::SE3Tangentd se3_jerk = (se3_acc_claw - se3_acc_t_1) / delta_T;


        ////////////// 速度加速度约束 ///////////////////
        manif::SE3Tangentd applied_acc = se3_acc_claw;

        if (se3_jerk.ang().norm() > rot_jerk_bound)
        {
            manif::SE3Tangentd applied_jerk = se3_jerk / (se3_jerk.ang().norm()) * rot_jerk_bound;
            applied_acc = se3_acc_t_1 + applied_jerk * delta_T;
        }
        manif::SE3Tangentd applied_vel = se3_vel_t_1 + applied_acc * delta_T;
        if (applied_acc.ang().norm() > rot_acc_bound)
        {
            applied_acc = applied_acc / applied_acc.ang().norm() * rot_acc_bound;
            applied_vel = se3_vel_t_1 + applied_acc * delta_T;
        }
        if (applied_vel.ang().norm() > rot_vec_bound)
        {
            applied_vel = applied_vel / applied_vel.ang().norm() * rot_vec_bound;
        }

        ////////////// 状态更新 ///////////////////
        se3_pos = se3_pos.rplus(applied_vel * delta_T);

        ////////////// 轨迹信息保存 ///////////////////
        // 1.保存 imu 数据
        se3_imu_v.push_back(tar_pose);
        // 2.保存 控制率 计算下发轨迹
        se3_pos_conLaw_v.push_back(se3_pos);
        // 3.保存 机器人监听数据
        se3_pos_Robot_v.push_back(manif::SE3d(0, 0, 0, robo_pose[3] * PI / 180, robo_pose[4] * PI / 180, robo_pose[5] * PI / 180) * imuCompensate.inverse());
       
        // 保存实际的加速度信息
        se3_acc_t_1 = (applied_vel - se3_vel_t_1) / delta_T;

        // 保存速度矢量
        se3_vel_t_1 = applied_vel;
        se3_Tarvel_t_1 = imu_vel;

        ////////////// 更新轨迹信息 ///////////////////
        manif::SE3d compensatedPose = se3_pos * imuCompensate;
        Eigen::Isometry3d curse3h = compensatedPose.isometry();
        Eigen::Vector3d rpy = GetRPY(curse3h);
        poseWaitForSend = {initPcsPos[0], initPcsPos[1], initPcsPos[2], rpy(0) * 180 / PI, rpy(1) * 180 / PI, rpy(2) * 180 / PI };


        ////////////// 等到消耗完时间片 ///////////////////
        while (1)
        {
            if ((clock() - begin) >= ((iter + 1) * delta_T * 1000))
            {
                break;
            }
            Sleep(0.00001);
        }
    }

    // 程序退出顺序表
    // 1. 更改标记位； 2. 关闭监听线程确保数据正确； 3. 断开数据链接，先imu后机器人。
    returnFlag = true;

    thread_IMU.join();
    HRIF_DisConnect(0);

    // 保存实际的位置信息
    std::string conLawBeforeTransfer = "SE3_P_control_conLaw_uniqueThread";
    savePath(conLawBeforeTransfer, se3_pos_conLaw_v, Time_v);

    // 保存实际的位置信息
    std::string imuBeforeTransfer = "SE3_P_control_imu_uniqueThread";
    savePath(imuBeforeTransfer, se3_imu_v, Time_v);

    // 写当前机器人的位置
    std::string robotBeforeTransfer = "SE3_P_control_robot_uniqueThread";
    savePath(robotBeforeTransfer, se3_pos_Robot_v, Time_v);

    // 写时间片
    std::string timeInterpert = "CommunicationTime";
    saveTime(timeInterpert, interpert_v);

    return 0;
}


void savePath(std::string fileName, std::vector<manif::SE3d> SE3_path, timeStamps time)
{
    // 写文件，把轨迹保存起来
    using namespace std;
    // 打开并写文件
    ofstream oFileM;

    std::string pathName = "./data/" + fileName + ".csv";
    oFileM.open(pathName, ios::out | ios::trunc);

    for (int i = 0; i < SE3_path.size(); i++)
    {
        manif::SE3d interp = SE3_path.at(i);
        oFileM << interp.x() << ","
            << interp.y() << ","
            << interp.z() << ","
            << interp.coeffs()(3) << ","
            << interp.coeffs()(4) << ","
            << interp.coeffs()(5) << ","
            << interp.coeffs()(6) << ","
            << time.at(i) << "\n";
    }
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

void saveHardwarePath(std::string fileName, std::vector<manif::SE3d> SE3_path, vector<double> ang_vel, timeStamps time)
{
    // 写文件，把轨迹保存起来
    using namespace std;
    // 打开并写文件
    ofstream oFileM;

    std::string pathName = "./data/" + fileName + ".csv";
    oFileM.open(pathName, ios::out | ios::trunc);

    for (int i = 0; i < SE3_path.size(); i++)
    {
        manif::SE3d interp = SE3_path.at(i);
        oFileM << interp.x() << ","
            << interp.y() << ","
            << interp.z() << ","
            << interp.coeffs()(3) << ","
            << interp.coeffs()(4) << ","
            << interp.coeffs()(5) << ","
            << interp.coeffs()(6) << ","
            << ang_vel.at(i) << ","
            << time.at(i) << "\n";
    }
}

