
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

// �����˵ĳ�ʼλ��
EcRealVector initRobotPos;
// �����˺�IMU����Բ�������̬
manif::SE3d imuCompensate;
// �����������Ŀ��λ��
manif::SE3d tar_pose;

// �����˵��ڲ�״̬
manif::SE3d se3_pos;

// �ȴ� servoP�·��� ��ʽ
EcRealVector poseWaitForSend;
// ʵ�������ÿ���ڼ�����״̬
EcRealVector robo_pose;
// ʵ�������ĩ��tcp��Է����̵�λ��
EcRealVector tcp;

// ��ʼ�ŷ�ʱ��
double delta_T;

// ��ױ���������� data_interpreter_state_keeper ����ʹ��
IMU_state_Interpreted imu_state;

/// �������߽����
double rot_vec_bound = PI;
double rot_acc_bound = 3;
double rot_jerk_bound = 30;

// ����Ŀ���������ģ�Ͳ���
double k_p = 2;
double k_v = 0.5;
double k_a = 0.01;

// ϵͳ�ڲ�����״̬�ı���
std::vector<manif::SE3d> se3_pos_conLaw_v;
std::vector<manif::SE3d> se3_imu_v;
std::vector<manif::SE3d> se3_pos_Robot_v;

std::vector<double> Time_v;
std::vector<double> angel_vel_diff_v;

// ϵͳ�ڲ��ı��������ڱ���ϵͳ�ٶ�״̬
// 4. ��̬ϵͳ��һʱ���ٶ�
manif::SE3Tangentd se3_vel_t_1 = manif::SE3Tangentd::Zero();
// 5. ������һ���ڵ�Ŀ���ٶ�
manif::SE3Tangentd se3_Tarvel_t_1 = manif::SE3Tangentd::Zero();
// 6. ������һ���ڵļ��ٶ�
manif::SE3Tangentd se3_acc_t_1 = manif::SE3Tangentd::Zero();
// 7. ���ٶȲ��
Eigen::Vector3d ang_d_diff;
// 8. �Ǽ��ٶȲ��
Eigen::Vector3d ang_dd_diff;

// ���ٶȱ������Ľ����� ���뵱ǰimu���ݣ����������λ�ýǶȺͽ��ٶ�
void data_interpreter_state_keeper(const EcRealVector& iData, manif::SE3d& pose, Eigen::Vector3d& ang_d, Eigen::Vector3d& ang_dd)
{
    // ��ȡ�Ƕ�
    double psi = iData[0] * PI / 180.;
    double theta = iData[1] * PI / 180.;
    double phi = iData[2] * PI / 180.;

    // ������̬
    Eigen::Matrix3d poseEi =
        (Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitX()).matrix() *
            Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).matrix() *
            Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()).matrix());

    Eigen::Isometry3d poseIso = Eigen::Isometry3d::Identity();
    poseIso.linear() = poseEi;
    manif::SE3d temp_pose = manif::SE3d(poseIso);


    if (temp_pose.isApprox(imu_state.pose))
    {
        // ʹ��֮ǰ���ٶȼ��ٶ�
        ang_d = imu_state.vel;
        ang_dd = imu_state.acc;
        pose = imu_state.pose;

        // ����Ҫ����
        return;
    }
    else
    {
        double time = imu_state.time;
        pose = temp_pose;
        // ���û�г�ʼ�����ٶ�Ϊ0�����ٶ�Ϊ0
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


// ��ʼ��̬���湦��
bool Elfin5_PoseTracker::trackerInitialized(const EcRealVector senseInitPose, const EcRealVector robInitPose, const double timeCycle)
{
    /////////////// �����ĳ�ʼ������ ///////////////
    if (senseInitPose.size() != 3) return false;

    // ����IMU�ĳ�ʼλ��
    manif::SE3d imuInitPose(0, 0, 0, senseInitPose[0] * PI / 180, senseInitPose[1] * PI / 180, senseInitPose[2] * PI / 180);
    tar_pose = imuInitPose;
    se3_pos = imuInitPose;

    //// ��ȡ�����˵�ǰTCP��һ�㣩��λ��
    //elfin.readPcsPos(initRobotPos);
    manif::SE3d robInitPoseSE(0, 0, 0, robInitPose[3] * PI / 180, robInitPose[4] * PI / 180, robInitPose[5] * PI / 180);

    //// ��ȡĩ��TCP
    //elfin.getTCP(tcp);

    // ����imuĿ�겹��λ��
    imuCompensate = imuInitPose.inverse() * robInitPoseSE;
    Eigen::Vector3d rpyInitCompensated = GetRPY((imuInitPose * imuCompensate).isometry());

    poseWaitForSend = { 0, 0, 0, rpyInitCompensated(0) * 180 / PI, rpyInitCompensated(1) * 180 / PI, rpyInitCompensated(2) * 180 / PI };

    // ��ʼ�����ŷ�����
    delta_T = timeCycle;
    //elfin.start_servo(indServoTime, indLookaheadTime);
    return true;
}

void Elfin5_PoseTracker::settingTrackerProfile(const double in_v_bound, const double in_a_bound, const double in_j_bound)
{
    /// ����Ŀ���������
    rot_vec_bound = in_v_bound;
    rot_acc_bound = in_a_bound;
    rot_jerk_bound = in_j_bound;
}

// ����һ����̬���ŷ��������̬ ��������һ���ڵĿ�����
void pushTargetPose(manif::SE3d imuTargetPose, manif::SE3d roboPoseSE3, Eigen::Vector3d ang_d_diff, Eigen::Vector3d ang_dd_diff)
{
    //cout << elfin.servoP(poseWaitForSend, tcp) << endl;
    //elfin.readPcsPos(robo_pose);

    //////////////// ϵͳ��״̬��ʼ�� /////////////////
 // ���������û����һʱ��λ�ã���˵���ڳ�ʱ�̣��ٶ�����Ϊ0��������ڳ�ʱ������ٶ�

    // �������� e_{R} = 1/2*��R_d^T*R - R^{T}*R_d��^{V}
    // �ٶȿ����� e_{omega} = omega - R^{T}*R_{d}*omage_{d}
    // ��������ٶ� \delta_{e} = -K_p * e_{R} - K_d* e_{omega};
    manif::SE3Tangentd angel_d;
    manif::SE3Tangentd angel_dd;

    angel_d.ang() = ang_d_diff - se3_vel_t_1.ang();
    angel_dd.ang() = ang_dd_diff - se3_acc_t_1.ang();

    manif::SE3Tangentd se3_vel_cLaw = k_p * imuTargetPose.rminus(roboPoseSE3) + k_v * (angel_d)+k_a * angel_dd;

    // ��������ʲ����ĽǼ��ٶ�W
    manif::SE3Tangentd se3_acc_claw = (se3_vel_cLaw - se3_vel_t_1) / delta_T;
    // ��������ʲ����ļӼ��ٶ�
    manif::SE3Tangentd se3_jerk = (se3_acc_claw - se3_acc_t_1) / delta_T;


    ////////////// �ٶȼ��ٶ�Լ�� ///////////////////
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

    ////////////// ״̬���� ///////////////////
    se3_pos = se3_pos.rplus(applied_vel * delta_T);

    ////////////// �켣��Ϣ���� ///////////////////
    // 1.���� imu ����
    se3_imu_v.push_back(tar_pose);
    // imu��ֵ��ٶȹ켣
    angel_vel_diff_v.push_back(ang_d_diff.norm());
    // 2.���� ������ �����·��켣
    se3_pos_conLaw_v.push_back(se3_pos);
    // 3.���� �����˼�������
    se3_pos_Robot_v.push_back(roboPoseSE3);

    // ����ʵ�ʵļ��ٶ���Ϣ
    se3_acc_t_1 = (applied_vel - se3_vel_t_1) / delta_T;

    // �����ٶ�ʸ��
    se3_vel_t_1 = applied_vel;

    ////////////// ���¹켣��Ϣ ///////////////////
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
