
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
#define RECORD_TIME 1000

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
    }

    return;
}

void savePath(std::string fileName, std::vector<manif::SE3d> SE3_path, timeStamps time);
void saveHardwarePath(std::string fileName, std::vector<manif::SE3d> SE3_path, vector<double> ang_vel, timeStamps time);

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


void RemoteServo(double indServoTime, double indLookaheadTime)
{
    //servoTime = 0.025   # 设置伺服周期为 25ms，建议最小不低于 15ms
    //	# 设置前瞻时间，前瞻时间越大，轨迹越平滑，但越滞后。
    //	lookaheadTime = 0.2 # 设置前瞻时间为 200ms, 建议在 0.05s~0.2s 之间
    std::vector<manif::SE3d> se3_pos_Robot_v;
    std::vector<manif::SE3d> se3_pos_conLaw_v;
    std::vector<manif::SE3d> se3_tarpose_v;
    std::vector<double> ang_vel_v;
    std::vector<double> cmdang_vel_v;
    std::vector<double> Time_v;

    conciseRobotController elfin;
    manif::SE3d pose;
    elfin.start_servo(indServoTime, indLookaheadTime);

    double begin; // 进度尺开始时间
    begin = clock();

    // 初始化伺服目标
    robot.lock();
    carKinInfo tempkinInfo = kinSharedInfo; 
    robot.unlock();
   
    int i = 0;

    EcRealVector pcs_lo;
    while(returnFlag == false)
    {
        robot.lock();
        carKinInfo tempkinInfo = kinSharedInfo; 
        robot.unlock();

        pcs_lo = tempkinInfo.pcs;
        elfin.servoP(pcs_lo);

        // 获取机器人 TCP 位置
        EcRealVector curPos;
        elfin.readPcsPos(curPos);

        //// 获取机器人关节速度
        double robAngVel;
        elfin.getAngularvelocity(robAngVel);

        // 记录imu的数据
        m.lock();
        imuDataPack iData_temp = iData;
        m.unlock();
        data_interpreter(iData_temp, pose);

        // 计算当前时间
        Time_v.push_back((clock() - begin) / 1000.);
        se3_pos_conLaw_v.push_back(manif::SE3d(pcs_lo[0], pcs_lo[1], pcs_lo[2], pcs_lo[3] * PI / 180, pcs_lo[4] * PI / 180, pcs_lo[5] * PI / 180));
        se3_pos_Robot_v.push_back(manif::SE3d(0, 0, 0, curPos[3] * PI / 180, curPos[4] * PI / 180, curPos[5] * PI / 180));
        se3_tarpose_v.push_back(pose);
        ang_vel_v.push_back(robAngVel);
        cmdang_vel_v.push_back(tempkinInfo.vel);

        while (1)
        {
            if ((clock() - begin) >= ((i + 1) * indServoTime * 1000))
            {
                break;
            }
            Sleep(0.001);
        }

        i++;
    }

    // 使用windows记录时刻信息
    std::string controlLawFileName = "SE3_P_control_path";
    saveHardwarePath(controlLawFileName, se3_pos_conLaw_v, cmdang_vel_v, Time_v);
    
    // 存储轨迹和时间文件信息
    std::string realDataFileName = "SE3_P_control_robot_path";
    saveHardwarePath(realDataFileName, se3_pos_Robot_v, ang_vel_v, Time_v);

    // 存储imu数据
    std::string imuDataFileName = "SE3_imuData";
    savePath(imuDataFileName, se3_tarpose_v, Time_v);
}

// 笛卡尔空间轨迹信息，包括速度，加速度


int main()
{
    //////////////// 数据容器 /////////////////
    // 保存的姿态轨迹信息
    std::vector<manif::SE3d> se3_pos_conLaw_v;
    std::vector<manif::SE3d> se3_tarpose_v;
    std::vector<manif::SE3d> se3_pos_Robot_v;

    std::vector<double> ang_vel_v;
    std::vector<double> Time_v;
    std::vector<double> T_v;

    //////////////// 基础参数信息/////////////////
    // 空置率参数
    double k_p = 1;


    // 对角速度，角加速度进行约束
    double rot_vec_bound = 0.5;
    double rot_acc_bound = 1;
    double rot_jerk_bound = 10;

    //////////////// 数据采集线程 /////////////////
    
    // 开启imu线程,数据开始被采集
    thread thread_IMU(imu_record_pose_info);
    returnFlag = false;
    
    //////////////// 机器人初始状态 /////////////////
    // 2. 动态系统上一时刻速度
    manif::SE3Tangentd se3_vel_t_1 = manif::SE3Tangentd::Zero();
    // 保存上一周期的目标速度
    manif::SE3Tangentd se3_Tarvel_t_1 = manif::SE3Tangentd::Zero();
    // 保存上一周期的加速度
    manif::SE3Tangentd se3_acc_t_1 = manif::SE3Tangentd::Zero();

    //////////////// 动态系统更新 /////////////////
    // 线程锁
    //std::lock_guard<std::mutex> lockGuard(m);

    int iter = 0; 

    //bool init_flag = false;

    double Times;

    HRIF_Connect(0, "10.0.0.27", 10003);
    // 新建控制
    conciseRobotController elfin;

    // 先运动到指定目标位置
    EcRealVector initJoint = {0, 0, 90, 0, 90, 0};
    elfin.moveJ(initJoint, 360, 50);

    // 获取初始位姿
    elfin.readPcsPos(initPcsPos);

    // 开始伺服控制
    double indServoTime = 0.03;
    double delta_T = indServoTime/3; // 2ms更新一次
    double indLookaheadTime = 0.05;

    double begin; // 进度尺开始时间
    begin = clock();

    EcRealVector curPos; // 机器人当前真实位置

    // 初始化机器人初始位姿
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

    manif::SE3d pose;

    // 1. 机器人初始状态
    Eigen::Isometry3d Tri = Eigen::Isometry3d::Identity();
    manif::SE3d se3_pos(imuInitPose);

    Eigen::Isometry3d initPoseCompensated = (imuInitPose * imuCompensate).isometry();
    Eigen::Vector3d rpyInitCompensated = GetRPY(initPoseCompensated);

    // 拼装机器人目标位置
    EcRealVector PoseInit = { initPcsPos[0], initPcsPos[1], initPcsPos[2], rpyInitCompensated(0) * 180 / PI, rpyInitCompensated(1) * 180 / PI, rpyInitCompensated(2) * 180 / PI };
    
    
    carKinInfo init_CarKinInfo;
    init_CarKinInfo.pcs = PoseInit;
    init_CarKinInfo.vel = 0;
    init_CarKinInfo.acc = 0;
    init_CarKinInfo.jerk = 0;

    ////// 更新servoP点位
    robot.lock();
    kinSharedInfo = init_CarKinInfo; // 获取本地数据
    robot.unlock();
    
    thread thread_Robot(RemoteServo, indServoTime, indLookaheadTime);

    //elfin.start_servo(indServoTime, indLookaheadTime);


    while (iter++ < RECORD_TIME)
    {
        //// 从内存池中获取本周期的状态 ////
        m.lock();
        imuDataPack iData_temp = iData;
        m.unlock();
        data_interpreter(iData_temp, pose); 

        //// 避免初状态异常 ////

        //// 本周期目标状态 ////
        manif::SE3d tar_pose = pose; 
        manif::SE3Tangentd tar_vel;


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
        manif::SE3Tangentd se3_vel_cLaw = tar_pose.rminus(se3_pos) * k_p + tar_vel;

        // 计算控制率产生的角加速度
        manif::SE3Tangentd se3_acc_claw = (se3_vel_cLaw - se3_vel_t_1) / delta_T;

        // 计算控制率产生的加加速度
        manif::SE3Tangentd se3_jerk = (se3_acc_claw - se3_acc_t_1) / delta_T;


        //// 如果加速度过大，对整个矢量进行截断
        manif::SE3Tangentd applied_acc = se3_acc_claw;

        if (se3_jerk.ang().norm() > rot_jerk_bound)
        {
            manif::SE3Tangentd applied_jerk = se3_jerk / (se3_jerk.ang().norm()) * rot_jerk_bound;

            //manif::SE2Tangentd applied_acc = manif::SE2Tangentd(se2_acc.x(), se2_acc.y(), sign(se2_acc.angle()) * rot_acc_bound);
            applied_acc = se3_acc_t_1 + applied_jerk * delta_T;
        }
        manif::SE3Tangentd applied_vel = se3_vel_t_1 + applied_acc * delta_T;
        if (applied_acc.ang().norm() > rot_acc_bound)
        {
            // 如果上一周期速度趋于变价
            applied_acc = applied_acc / applied_acc.ang().norm() * rot_acc_bound;

            //manif::SE2Tangentd applied_acc = manif::SE2Tangentd(se2_acc.x(), se2_acc.y(), sign(se2_acc.angle()) * rot_acc_bound);
            applied_vel = se3_vel_t_1 + applied_acc * delta_T;
        }
        // 如果速度还过大，约束速度
        if (applied_vel.ang().norm() > rot_vec_bound)
        {
            // 判断加速度是否为 0， 如果不为0 使用最大jerk降速到速度边界内

            applied_vel = applied_vel / applied_vel.ang().norm() * rot_vec_bound;
            // 重新算一下jerk是否超了，如果超了约束住jerk
            //if(jerk)
        }

        // 更新机器人位置
        se3_pos = se3_pos.rplus(applied_vel * delta_T);

        //// 保存轨迹信息
        //se3_pos_conLaw_v.push_back(se3_pos* imuCompensate);
        se3_tarpose_v.push_back(tar_pose);
        se3_pos_conLaw_v.push_back(se3_pos);
        Time_v.push_back((clock() - begin) / 1000.);

        // 保存实际的加速度信息
        se3_acc_t_1 = (applied_vel - se3_vel_t_1) / delta_T;

        // 保存速度矢量
        se3_vel_t_1 = applied_vel;
        se3_Tarvel_t_1 = tar_vel;

        // 拼装下发指令 imuInitPose.inverse()* robInitPose (这里应该是不动的)
        manif::SE3d compensatedPose = se3_pos * imuCompensate;
        Eigen::Isometry3d curse3h = compensatedPose.isometry();
        Eigen::Vector3d rpy = GetRPY(curse3h);

        // 拼装机器人目标位置
        EcRealVector cmdPose = {initPcsPos[0], initPcsPos[1], initPcsPos[2], rpy(0) * 180 / PI, rpy(1) * 180 / PI, rpy(2) * 180 / PI };

        carKinInfo temp_CarKinInfo;
        temp_CarKinInfo.pcs = cmdPose;
        temp_CarKinInfo.vel = tar_vel.ang().norm();
        temp_CarKinInfo.acc = se3_acc_t_1.ang().norm();
        temp_CarKinInfo.jerk = 0;

        ////// 更新servoP点位
        robot.lock();
        kinSharedInfo = temp_CarKinInfo; // 获取本地数据
        robot.unlock();

        //{
        //    cout << elfin.servoP(cmdPose) << "\n";

        //    // 获取机器人 TCP 位置
        //    EcRealVector curPos;
        //    elfin.readPcsPos(curPos);

        //    //// 获取机器人 TCP 速度
        //    double angVel;
        //    elfin.getAngularvelocity(angVel);
        //    ang_vel_v.push_back(angVel);

        //    // 计算当前时间
        //    Time_v.push_back((clock() - begin) / 1000.);
        //    //se3_pos_conLaw_v.push_back(manif::SE3d(cmdPose[0], cmdPose[1], cmdPose[2], cmdPose[3] * PI / 180, cmdPose[4] * PI / 180, cmdPose[5] * PI / 180));
        //    se3_pos_Robot_v.push_back(manif::SE3d(0, 0, 0, curPos[3] * PI / 180, curPos[4] * PI / 180, curPos[5] * PI / 180));
        //    ang_vel_v.push_back(angVel);
        //}


        //elfin.servoP(cmdPose); // 发送 servoP 点位
        //Time_v.push_back((clock() - begin) / 1000.); // 记录下发时刻时间

        while (1)
        {
            if ((clock() - begin) >= ((iter + 1) * delta_T * 1000))
            {
                break;
            }
            Sleep(0.001);
        }
    }

    // 程序退出顺序表
    // 1. 更改标记位； 2. 关闭监听线程确保数据正确； 3. 断开数据链接，先imu后机器人。
    returnFlag = true;

    thread_Robot.join();
    thread_IMU.join();
    HRIF_DisConnect(0);

    // 保存实际的位置信息
    std::string conLawBeforeTransfer = "SE3_P_control_conLaw_bTrans";
    savePath(conLawBeforeTransfer, se3_pos_conLaw_v, Time_v);

    // 保存实际的位置信息
    std::string imuBeforeTransfer = "SE3_P_control_imu_bTrans";
    savePath(imuBeforeTransfer, se3_tarpose_v, Time_v);

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

