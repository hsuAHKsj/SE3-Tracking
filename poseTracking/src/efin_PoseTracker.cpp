
#include <Eigen/Dense>
#include "manif/SO3.h"

#include <hansTracker/elfin_PoseTracker.h>
#include "manif/SE3.h"

struct IMU_state_Interpreted
{
    manif::SE3d pose = manif::SE3d::Identity();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    double time = 0;
};

// 机器人的初始位姿
EcRealVector initRobotPos;
// 机器人和IMU的相对补偿的姿态
manif::SE3d imuCompensate;
// 控制器解算的目标位姿
manif::SE3d tar_pose;

// 机器人的内部状态
manif::SE3d se3_pos;

// 等待 servoP下发的 格式
EcRealVector poseWaitForSend;
// 实体机器人每周期监听的状态
EcRealVector robo_pose;
// 实体机器人末端tcp相对法郎盘的位置
EcRealVector tcp;

// 开始伺服时刻
double delta_T;

// 零阶保持器，结合 data_interpreter_state_keeper 函数使用
IMU_state_Interpreted imu_state;

/// 控制器边界参数
double rot_vec_bound = PI;
double rot_acc_bound = 3;
double rot_jerk_bound = 30;

// 具体的控制器二阶模型参数
double k_p = 2;
double k_v = 0.5;
double k_a = 0.01;

// 系统内部保存状态的变量
std::vector<manif::SE3d> se3_pos_conLaw_v;
std::vector<manif::SE3d> se3_imu_v;
std::vector<manif::SE3d> se3_pos_Robot_v;

std::vector<double> Time_v;
std::vector<double> angel_vel_diff_v;

// 系统内部的变量，用于保存系统速度状态
// 4. 动态系统上一时刻速度
manif::SE3Tangentd se3_vel_t_1 = manif::SE3Tangentd::Zero();
// 5. 保存上一周期的目标速度
manif::SE3Tangentd se3_Tarvel_t_1 = manif::SE3Tangentd::Zero();
// 6. 保存上一周期的加速度
manif::SE3Tangentd se3_acc_t_1 = manif::SE3Tangentd::Zero();
// 7. 角速度差分
Eigen::Vector3d ang_d_diff;
// 8. 角加速度差分
Eigen::Vector3d ang_dd_diff;

// 带速度保持器的解释器 输入当前imu数据，输出机器人位置角度和角速度
void data_interpreter_state_keeper(const EcRealVector& iData, manif::SE3d& pose, Eigen::Vector3d& ang_d, Eigen::Vector3d& ang_dd)
{
    // 获取角度
    double psi = iData[0] * PI / 180.;
    double theta = iData[1] * PI / 180.;
    double phi = iData[2] * PI / 180.;

    // 计算姿态
    Eigen::Matrix3d poseEi =
        (Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitX()).matrix() *
            Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).matrix() *
            Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()).matrix());

    Eigen::Isometry3d poseIso = Eigen::Isometry3d::Identity();
    poseIso.linear() = poseEi;
    manif::SE3d temp_pose = manif::SE3d(poseIso);


    if (temp_pose.isApprox(imu_state.pose))
    {
        // 使用之前的速度加速度
        ang_d = imu_state.vel;
        ang_dd = imu_state.acc;
        pose = imu_state.pose;

        // 不需要保存
        return;
    }
    else
    {
        double time = imu_state.time;
        pose = temp_pose;
        // 如果没有初始化，速度为0，加速度为0
        if (imu_state.pose.isApprox(manif::SE3d::Identity()))
        {
            ang_d = Eigen::Vector3d::Zero();
        }
        else
        {
            manif::SE3Tangentd temAnd_d = pose.rminus(imu_state.pose);
            ang_d = temAnd_d.ang() / ((clock() - time) / 1000.);
        }

        ang_dd = (ang_d - imu_state.vel) / ((clock() - time) / 1000.);

        imu_state.pose = pose;
        imu_state.vel = ang_d;
        imu_state.acc = ang_dd;
        imu_state.time = clock();
        return;
    }
}

// Gives back a rotation matrix specified with RPY convention
Eigen::Vector3d GetRPY(Eigen::Isometry3d iso)
{
    Eigen::Matrix3d data = iso.linear();
    Eigen::Vector3d outVec;
    double epsilon = 1E-12;
    outVec(1) = atan2(-data(2, 0), sqrt(data(0, 0) * data(0, 0) + data(1, 0) * data(1, 0)));
    if (fabs(outVec(1)) > (PI / 2.0 - epsilon)) {
        outVec(2) = atan2(-data(0, 1), data(1, 1));
        outVec(0) = 0.0;
    }
    else {
        outVec(0) = atan2(data(2, 1), data(2, 2));
        outVec(2) = atan2(data(1, 0), data(0, 0));
    }

    return outVec;
}


// 开始姿态跟随功能
bool Elfin5_PoseTracker::trackerInitialized(const EcRealVector senseInitPose, const EcRealVector robInitPose, const double timeCycle)
{
    /////////////// 变量的初始化操作 ///////////////
    if (senseInitPose.size() != 3) return false;

    // 计算IMU的初始位姿
    manif::SE3d imuInitPose(0, 0, 0, senseInitPose[0] * PI / 180, senseInitPose[1] * PI / 180, senseInitPose[2] * PI / 180);
    tar_pose = imuInitPose;
    se3_pos = imuInitPose;

    //// 获取机器人当前TCP（一般）的位姿
    //elfin.readPcsPos(initRobotPos);
    manif::SE3d robInitPoseSE(0, 0, 0, robInitPose[3] * PI / 180, robInitPose[4] * PI / 180, robInitPose[5] * PI / 180);

    //// 获取末端TCP
    //elfin.getTCP(tcp);

    // 计算imu目标补偿位姿
    imuCompensate = imuInitPose.inverse() * robInitPoseSE;
    Eigen::Vector3d rpyInitCompensated = GetRPY((imuInitPose * imuCompensate).isometry());

    poseWaitForSend = { 0, 0, 0, rpyInitCompensated(0) * 180 / PI, rpyInitCompensated(1) * 180 / PI, rpyInitCompensated(2) * 180 / PI };

    // 开始启动伺服控制
    delta_T = timeCycle;
    //elfin.start_servo(indServoTime, indLookaheadTime);
    return true;
}

void Elfin5_PoseTracker::settingTrackerProfile(const double in_v_bound, const double in_a_bound, const double in_j_bound)
{
    /// 具体的控制器参数
    rot_vec_bound = in_v_bound;
    rot_acc_bound = in_a_bound;
    rot_jerk_bound = in_j_bound;
}

// 推入一个姿态，伺服到这个姿态 并计算下一周期的控制率
void pushTargetPose(manif::SE3d imuTargetPose, manif::SE3d roboPoseSE3, Eigen::Vector3d ang_d_diff, Eigen::Vector3d ang_dd_diff)
{
    //cout << elfin.servoP(poseWaitForSend, tcp) << endl;
    //elfin.readPcsPos(robo_pose);

    //////////////// 系统初状态初始化 /////////////////
 // 如果池子里没有上一时刻位置，则说明在初时刻，速度设置为0；如果不在初时刻求解速度

    // 误差控制率 e_{R} = 1/2*（R_d^T*R - R^{T}*R_d）^{V}
    // 速度控制率 e_{omega} = omega - R^{T}*R_{d}*omage_{d}
    // 整体控制速度 \delta_{e} = -K_p * e_{R} - K_d* e_{omega};
    manif::SE3Tangentd angel_d;
    manif::SE3Tangentd angel_dd;

    angel_d.ang() = ang_d_diff - se3_vel_t_1.ang();
    angel_dd.ang() = ang_dd_diff - se3_acc_t_1.ang();

    manif::SE3Tangentd se3_vel_cLaw = k_p * imuTargetPose.rminus(roboPoseSE3) + k_v * (angel_d)+k_a * angel_dd;

    // 计算控制率产生的角加速度W
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
    // imu差分的速度轨迹
    angel_vel_diff_v.push_back(ang_d_diff.norm());
    // 2.保存 控制率 计算下发轨迹
    se3_pos_conLaw_v.push_back(se3_pos);
    // 3.保存 机器人监听数据
    se3_pos_Robot_v.push_back(roboPoseSE3);

    // 保存实际的加速度信息
    se3_acc_t_1 = (applied_vel - se3_vel_t_1) / delta_T;

    // 保存速度矢量
    se3_vel_t_1 = applied_vel;

    ////////////// 更新轨迹信息 ///////////////////
    manif::SE3d compensatedPose = se3_pos * imuCompensate;
    Eigen::Isometry3d curse3h = compensatedPose.isometry();
    Eigen::Vector3d rpy = GetRPY(curse3h);
    poseWaitForSend = { 0, 0, 0, rpy(0) * 180 / PI, rpy(1) * 180 / PI, rpy(2) * 180 / PI };
};

bool Elfin5_PoseTracker::updateTargetPose(const EcRealVector targetPose, const EcRealVector robot_pose)
{
    if (targetPose.size() != 3 && robot_pose.size() != 6) return false;
    manif::SE3d roboPoseSE3 = manif::SE3d(0, 0, 0, robot_pose[3] * PI / 180, robot_pose[4] * PI / 180, robot_pose[5] * PI / 180) * imuCompensate.inverse();
    manif::SE3d imuTargetPose;
    data_interpreter_state_keeper(targetPose, imuTargetPose, ang_d_diff, ang_dd_diff);
    pushTargetPose(imuTargetPose, roboPoseSE3, ang_d_diff, ang_dd_diff);
    return true;
};

void Elfin5_PoseTracker::getDeltaPose(EcRealVector& deltaPose)
{
    EcRealVector resPose = { poseWaitForSend[3], poseWaitForSend[4], poseWaitForSend[5] };
    deltaPose = resPose;
}
