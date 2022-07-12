
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h> 
#include <Eigen/Core>
#include <Eigen/Dense>
#include "manif/SO3.h"

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

// �����̶߳�ȡ������λ����Ϣ
void imu_record_pose_info()
{
    imu_com imu;

    //std::lock_guard<std::mutex> lockGuard(m);
    // ����imu��Ϣ�����浽ȫ�ֱ�������
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
Eigen::Vector3d Vee(const Eigen::Matrix3d& mat);

void save_time(std::string fileName, std::vector<double>& T_v)
{
    // д�ļ����ѹ켣��������
    using namespace std;
    // �򿪲�д�ļ�
    ofstream oFileM;

    std::string pathName = "./data/" + fileName + ".csv";
    oFileM.open(pathName, ios::out | ios::trunc);

    for (const auto& Time : T_v)
    {
        oFileM << Time << "\n";
    }
}


void data_interpreter(const imuDataPack& iData, manif::SE3d& pose, Eigen::Vector3d& ang_d, Eigen::Vector3d& ang_dd)
{
    // ��ȡ�Ƕ�
    double psi = iData.angle[0] * PI / 180.;
    //���Բ�Ʒ�ֲ� ��Ʒ�����Ƕ���ŷ���ǵģ�ŷ���ǵ���ת˳����zyx˳��

    double theta = iData.angle[1] * PI / 180.;
    double phi = iData.angle[2] * PI / 180.;

    //cout << " IMU Data : " << iData.angle[0] << ", " << iData.angle[1] << ", " << iData.angle[2] << endl;

    // ��̬
    Eigen::Matrix3d poseEi =
        (Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitX()).matrix() *
            Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).matrix() *
            Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()).matrix());

    Eigen::Isometry3d poseIso = Eigen::Isometry3d::Identity();
    poseIso.linear() = poseEi;
    pose = manif::SE3d(poseIso);

    // �����ٶ�ת���ɵ�ȫ������ϵ��
    Eigen::Matrix3d Eul2AngMat;
    Eul2AngMat << 1, 0, -sin(theta),
        0, cos(psi), sin(psi)* cos(theta),
        0, -sin(psi), cos(psi)* cos(theta);
    
    Eigen::Vector3d eular_d(iData.w[0], iData.w[1], iData.w[2]);
    Eigen::Vector3d eular_dd(iData.a[0], iData.a[1], iData.a[2]);


    // �����Ϣ
    ang_d =  poseEi * eular_d /180 *PI;
    ang_dd = poseEi * eular_dd / 180 * PI;
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
    //////////////// �������� /////////////////
    // �������̬�켣��Ϣ
    std::vector<manif::SE3d> se3_pos_conLaw_v;
    std::vector<manif::SE3d> se3_imu_v;
    std::vector<manif::SE3d> se3_pos_Robot_v;
    
    std::vector<double> ang_vel_v;
    std::vector<double> Time_v;
    std::vector<double> interpert_v;
    std::vector<double> angel_vel_v;


    //////////////// ����������Ϣ/////////////////
    // �����ʲ���
    double k_p = 1;
    double k_v = 0.1;

    // �Խ��ٶȣ��Ǽ��ٶȽ���Լ��
    double rot_vec_bound = PI;
    double rot_acc_bound = 5;
    double rot_jerk_bound = 10;

    //////////////// ���ݲɼ��߳� /////////////////
    
    // ����imu�߳�,���ݿ�ʼ���ɼ�
    thread thread_IMU(imu_record_pose_info);
   
    /////////////// ���ӻ����ˣ��˶�����ʼλ�� ///////////////
    HRIF_Connect(0, "10.0.0.27", 10003);
    // �½�����
    conciseRobotController elfin;
    // ���˶���ָ��Ŀ��λ��
    EcRealVector initJoint = {0, 0, 90, 0, 90, 0};

    elfin.moveJ(initJoint, 360, 50);

    /////////////// ��ȡIMU�ͻ����˵ĳ�ʼλ�ˣ������㲹��λ�� ///////////////
    elfin.readPcsPos(initPcsPos);
    manif::SE3d robInitPose(0, 0, 0, initPcsPos[3] * PI / 180, initPcsPos[4] * PI / 180, initPcsPos[5] * PI / 180);

    // ��ȡimu��ʼλ��
    manif::SE3d imuInitPose;
    Eigen::Vector3d ang_d;
    Eigen::Vector3d ang_dd;
    do
    {
        m.lock();
        imuDataPack temImuData = iData;
        m.unlock();
        // ���ڴ���л�ȡ
        data_interpreter(temImuData, imuInitPose, ang_d, ang_dd);
        Sleep(1000);
    } while (imuInitPose.isApprox(manif::SE3d::Identity()));


    // ����imuĿ�겹��λ��
    manif::SE3d imuCompensate =  imuInitPose.inverse()* robInitPose;
    Eigen::Vector3d rpyInitCompensated = GetRPY((imuInitPose * imuCompensate).isometry());

    /////////////// ��ʼ��������ѭ���еı��� ///////////////
    
    // 1. �ȴ�ServoP���͵�ָ����Ϣ
    EcRealVector poseWaitForSend = { initPcsPos[0], initPcsPos[1], initPcsPos[2], rpyInitCompensated(0) * 180 / PI, rpyInitCompensated(1) * 180 / PI, rpyInitCompensated(2) * 180 / PI };
    // 2. �������ڲ��ĵ�ǰ��Ϣ
    manif::SE3d se3_pos(imuInitPose);
    // 3. IMU��ȡ��Ŀ��״̬
    manif::SE3d tar_pose;
    // 4. ʵ������˼����õ���״̬
    EcRealVector robo_pose;

    //////////////// ��һʱ�̵���Ϣ״̬��Ϣ /////////////////
    // 4. ��̬ϵͳ��һʱ���ٶ�
    manif::SE3Tangentd se3_vel_t_1 = manif::SE3Tangentd::Zero();
    // 5. ������һ���ڵ�Ŀ���ٶ�
    manif::SE3Tangentd se3_Tarvel_t_1 = manif::SE3Tangentd::Zero();
    // 6. ������һ���ڵļ��ٶ�
    manif::SE3Tangentd se3_acc_t_1 = manif::SE3Tangentd::Zero();
    // 7. Ŀ���ٶ�
    manif::SE3Tangentd imu_vel;
    // 8. ��һ���ڵ�����ģ
    double errorNorm_t_1 = 0;

    // ��ʼ�ŷ�����
    double indServoTime = 0.05;
    double delta_T = indServoTime; // 2ms����һ��
    double indLookaheadTime = 0.05;
    returnFlag = false;
    int iter = 0;

    // ����servoָ���ʼ��ʱ
    elfin.start_servo(indServoTime, indLookaheadTime);
    double begin = clock();

    //////////////// ��̬ϵͳ���� /////////////////
    while (iter++ < RECORD_TIME)
    {
        //////////////// �·� servoP ָ�� /////////////////
        double count_tick = clock();
        cout << elfin.servoP(poseWaitForSend) << endl;
        elfin.readPcsPos(robo_pose);

        manif::SE3d roboPoseSE3 = manif::SE3d(0, 0, 0, robo_pose[3] * PI / 180, robo_pose[4] * PI / 180, robo_pose[5] * PI / 180)* imuCompensate.inverse();

        interpert_v.push_back((clock() - count_tick) / 1000.);
        Time_v.push_back((clock() - begin) / 1000.);

        //////////////// ��̬ϵͳ���� /////////////////
        m.lock();
        imuDataPack iData_temp = iData;
        m.unlock();

        data_interpreter(iData_temp, tar_pose, ang_d, ang_dd);
        //cout << "ang_d = " << ang_d << endl;
        angel_vel_v.push_back(ang_d.norm());

        //////////////// ϵͳ��״̬��ʼ�� /////////////////
        // ���������û����һʱ��λ�ã���˵���ڳ�ʱ�̣��ٶ�����Ϊ0��������ڳ�ʱ������ٶ�
        if (se3_imu_v.size() == 0)
        {
            imu_vel = manif::SE3Tangentd::Zero();
        }
        else
        {
            // ������ݸ���һʱ�����ݷǳ��ӽ�������ʹ��0 ���ٶȵĹ����˶�
            // ���λ�ò�÷ǳ�Զ�������·��������Ŀ��Ҳ���п����ǲ�������

            if (tar_pose.isApprox(*se3_imu_v.rbegin()))
            {
                imu_vel = se3_Tarvel_t_1;
            }
            else
            {
                imu_vel = (tar_pose.rminus(*se3_imu_v.rbegin())) / delta_T;
            }
        }

        // ����Ŀ����ٶ�
        manif::SE3Tangentd imu_acc;
        if (se3_Tarvel_t_1.isApprox(manif::SE3Tangentd::Zero()))
        {
            imu_acc = manif::SE3Tangentd::Zero();
        }
        else{
            imu_acc = (imu_vel - se3_Tarvel_t_1) / delta_T;
        }


        // �������� e_{R} = 1/2*��R_d^T*R - R^{T}*R_d��^{V}
        // �ٶȿ����� e_{omega} = omega - R^{T}*R_{d}*omage_{d}
        // ��������ٶ� \delta_{e} = -K_p * e_{R} - K_d* e_{omega};
        manif::SE3Tangentd angel_d;
        manif::SE3Tangentd angel_dd;

        angel_d.ang() = ang_d - se3_vel_t_1.ang();
        angel_dd.ang() = ang_dd;

        manif::SE3Tangentd se3_acc_claw = 5*tar_pose.rminus(roboPoseSE3) * k_p + 0.2*(angel_d) + 0.1 *(angel_dd - se3_acc_t_1);
        // ��������ʲ����Ľ��ٶ�
        manif::SE3Tangentd se3_vel_cLaw = se3_vel_t_1 + se3_acc_claw * delta_T;
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
        // 2.���� ������ �����·��켣
        se3_pos_conLaw_v.push_back(se3_pos);
        // 3.���� �����˼�������
        se3_pos_Robot_v.push_back(roboPoseSE3);
       
        // ����ʵ�ʵļ��ٶ���Ϣ
        se3_acc_t_1 = (applied_vel - se3_vel_t_1) / delta_T;

        // �����ٶ�ʸ��
        se3_vel_t_1 = applied_vel;
        se3_Tarvel_t_1 = imu_vel;

        ////////////// ���¹켣��Ϣ ///////////////////
        manif::SE3d compensatedPose = se3_pos * imuCompensate;
        Eigen::Isometry3d curse3h = compensatedPose.isometry();
        Eigen::Vector3d rpy = GetRPY(curse3h);
        poseWaitForSend = {initPcsPos[0], initPcsPos[1], initPcsPos[2], rpy(0) * 180 / PI, rpy(1) * 180 / PI, rpy(2) * 180 / PI };


        ////////////// �ȵ�������ʱ��Ƭ ///////////////////
        while (1)
        {
            if ((clock() - begin) >= ((iter + 1) * delta_T * 1000))
            {
                break;
            }
            Sleep(0.00001);
        }
    }

    // �����˳�˳���
    // 1. ���ı��λ�� 2. �رռ����߳�ȷ��������ȷ�� 3. �Ͽ��������ӣ���imu������ˡ�
    returnFlag = true;

    thread_IMU.join();
    HRIF_DisConnect(0);

    // ����ʵ�ʵ�λ����Ϣ
    std::string conLawBeforeTransfer = "SE3_P_control_conLaw_uniqueThread";
    savePath(conLawBeforeTransfer, se3_pos_conLaw_v, Time_v);

    // ����ʵ�ʵ�λ����Ϣ
    std::string imuBeforeTransfer = "SE3_P_control_imu_uniqueThread";
    savePath(imuBeforeTransfer, se3_imu_v, Time_v);

    // д��ǰ�����˵�λ��
    std::string robotBeforeTransfer = "SE3_P_control_robot_uniqueThread";
    savePath(robotBeforeTransfer, se3_pos_Robot_v, Time_v);

    // дʱ��Ƭ
    std::string timeInterpert = "CommunicationTime";
    saveTime(timeInterpert, interpert_v);

    // �������ٶ�
    std::string angleVel = "angleVel";
    saveTime(angleVel, angel_vel_v);

    return 0;
}

Eigen::Vector3d Vee(const Eigen::Matrix3d& mat)
{
    Eigen::Vector3d t; 
    t << mat(0, 2), mat(2,1), mat(1, 0);
    return t;
}

Eigen::Vector3d saveAngelVel(std::string fileName, std::vector<double> vel_V)
{
    Eigen::Vector3d t;
    ofstream oFileM;

    std::string pathName = "./data/" + fileName + ".csv";
    oFileM.open(pathName, ios::out | ios::trunc);
    for (int i = 0; i < vel_V.size(); i++)
    {
        oFileM << vel_V.at(i) << "\n";
    }
    return t;
}

void savePath(std::string fileName, std::vector<manif::SE3d> SE3_path, timeStamps time)
{
    // д�ļ����ѹ켣��������
    using namespace std;
    // �򿪲�д�ļ�
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
    // д�ļ����ѹ켣��������
    using namespace std;
    // �򿪲�д�ļ�
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
    // д�ļ����ѹ켣��������
    using namespace std;
    // �򿪲�д�ļ�
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

//Eigen::Matrix3d R_d = tar_pose.rotation();
//Eigen::Matrix3d R = se3_pos.rotation();
//Eigen::Vector3d omega_d = imu_vel.ang();
//Eigen::Vector3d omega = se3_vel_t_1.ang();
//Eigen::Vector3d e_omega = omega - R.transpose() * R_d * omega_d;
//Eigen::Vector3d e_R = Vee(R_d.transpose() * R - R.transpose() * R_d)/2;
//manif::SO3Tangentd e_omega_so3(e_omega);
//manif::SO3Tangentd e_R_so3(e_R);

//cout << "e_R = " << e_R << endl;
//cout << "R = " << R << endl;



//manif::SO3Tangentd geo_screw = R.transpose() * (-k_p * e_R_so3);
//cout << "geo_screw = " << geo_screw << endl;

//manif::SE3Tangentd se3_vel_cLaw = manif::SE3Tangentd::Zero();
//se3_vel_cLaw.ang() = geo_screw.ang();
//manif::SE3Tangentd se3_vel_cLaw = tar_pose.rminus(se3_pos) * k_p - se3_e_omega * k_v;